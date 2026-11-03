#!/usr/bin/env python3
"""Receive FPGA video and return YOLO or VLM target measurements to the FPGA.

The network thread continuously drains UDP while inference consumes only the
newest complete frame. --no-inference needs only the Python standard library.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import queue
import socket
import sys
import threading
import time
from pathlib import Path

from dataclasses import replace
from collections import OrderedDict

from protocol import (MAX_BOXES, OBSERVED, VIDEO_HEIGHT, VIDEO_WIDTH,
                      DetectionBox, FrameAssembler, ProtocolError, VideoFrame,
                      box_from_xyxy, decode_video_line, encode_boxes)
from tracking import (MAX_EXACT_TRACK_ID, LockMailbox, TrackingState,
                      choose_reply_boxes, evict_track_ids)
from model_config import COCO_NAMES, model_class_mapping, wire_class_id
from task_commands import MOTION, TaskController, parse_text
from thermal_monitor import ThermalMonitor
from thermal_task_gate import ThermalTaskGate
from thermal_preview_service import stop_preview_service
from vlm_tasks import (VlmClient, VlmTaskController, VLM_WIRE_CLASS,
                       YoloObservation, parse_visual_goal)

LOG = logging.getLogger("fpga_yolo")
SCRIPT_DIRECTORY = Path(__file__).resolve().parent


class LatestFrame:
    """A one-slot mailbox: inference never builds a queue of old images."""

    def __init__(self):
        self._condition = threading.Condition()
        self._frame: VideoFrame | None = None
        self.replaced = 0

    def publish(self, frame: VideoFrame):
        with self._condition:
            if self._frame is not None:
                self.replaced += 1
            self._frame = frame
            self._condition.notify()

    def take(self, timeout=0.2) -> VideoFrame | None:
        with self._condition:
            self._condition.wait_for(lambda: self._frame is not None, timeout)
            frame = self._frame
            self._frame = None
            return frame


class UdpVideoReceiver(threading.Thread):
    """The single network socket receives video and sends boxes from port 5000."""

    def __init__(self, udp_socket, latest, fpga_ip, frame_timeout=0.25,
                 sequence_reset_timeout=2.0, locks=None):
        super().__init__(name="udp-video-rx", daemon=True)
        self.udp_socket = udp_socket
        self.latest = latest
        self.fpga_ip = fpga_ip
        self.locks = locks if locks is not None else LockMailbox()
        self.stop_event = threading.Event()
        self.failure: Exception | None = None
        self.rejected_sources = 0
        self.last_source = None
        self.last_frame_at = None
        from screen_recorder import ScreenAssembler
        self.screen_assembler = ScreenAssembler(frame_timeout=frame_timeout,
                                        sequence_reset_timeout=sequence_reset_timeout)
        self.assembler = FrameAssembler(frame_timeout=frame_timeout,
                                        sequence_reset_timeout=sequence_reset_timeout,
                                        auto_geometry=True)

    def run(self):
        try:
            while not self.stop_event.is_set():
                try:
                    packet, address = self.udp_socket.recvfrom(2048)
                except socket.timeout:
                    self.assembler.expire()
                    self.screen_assembler.expire()
                    continue
                except OSError as error:
                    # Linux truncates an oversized datagram; Windows reports WSAEMSGSIZE.
                    # In both cases an invalid datagram must not stop the receiver.
                    if getattr(error, "winerror", None) == 10040:
                        self.assembler.stats["malformed"] += 1
                        continue
                    raise
                if address[0] != self.fpga_ip:
                    self.rejected_sources += 1
                    continue
                if packet.startswith(b"FAUD"):
                    # Task 6 uses text sent over Wi-Fi, not FPGA audio/Vosk.
                    continue
                now = time.monotonic()
                if packet.startswith(b"FREC"):
                    frame = self.screen_assembler.feed(packet, now)
                    if frame is not None:
                        if self.last_source != "hdmi":
                            LOG.info("HDMI stream: FREC v%d %s %dx%d", self.screen_assembler.wire_version,
                                     "RGB565BE" if self.screen_assembler.wire_format == 1 else "RGB332",
                                     frame.width, frame.height)
                        self.last_source = "hdmi"
                        self.last_frame_at = now
                        self.latest.publish(replace(frame, source="hdmi"))
                    continue
                try:
                    line = decode_video_line(packet, *(self.assembler.geometry or (None, None)))
                except ProtocolError:
                    self.assembler.stats["packets"] += 1
                    self.assembler.stats["malformed"] += 1
                    self.assembler.expire(now)
                    continue
                malformed_before = self.assembler.stats['malformed']
                frame = self.assembler.feed_line(line, now)
                if self.assembler.stats['malformed'] != malformed_before:
                    continue
                # Do not wait for all 360 rows to update the selected-track hint.
                self.locks.update(line, self.assembler.stats["sequence_resets"], now)
                if frame is not None:
                    self.last_source = "camera"
                    self.last_frame_at = now
                    self.latest.publish(frame)
        except OSError as error:
            if not self.stop_event.is_set():
                self.failure = error
                LOG.exception("UDP receive thread failed")
        except Exception as error:
            self.failure = error
            LOG.exception("video assembly failed")

    def stop(self):
        self.stop_event.set()
        self.join(timeout=1.0)


def rgb565_to_bgr(frame: VideoFrame):
    """RGB565 wire pixels are high byte first; Ultralytics ndarray input is BGR."""
    import numpy as np

    words = np.frombuffer(frame.pixels, dtype=">u2").reshape(frame.height, frame.width)
    blue = words & 31
    green = (words >> 5) & 63
    red = (words >> 11) & 31
    # Expand 5/6-bit channels by bit replication, preserving white as 255.
    image = np.empty((frame.height, frame.width, 3), dtype=np.uint8)
    image[:, :, 0] = (blue << 3) | (blue >> 2)
    image[:, :, 1] = (green << 2) | (green >> 4)
    image[:, :, 2] = (red << 3) | (red >> 2)
    return image


class ObjectDetector:
    def __init__(self, model_path: Path, confidence: float, iou: float, threads: int,
                 tracker="botsort", candidate_confidence=0.1, candidate_limit=64,
                 track_retention=2.0, track_max_gap=0.6, reacquire=True):
        try:
            import cv2
            import ncnn  # noqa: F401; fail now instead of installing during a live stream.
            import numpy as np
            import torch
            import yaml
            import ultralytics
            from ultralytics import YOLO
            if tracker != "none":
                import lap  # noqa: F401; no live-stream dependency auto-install.
                import scipy  # noqa: F401
        except ImportError as error:
            raise RuntimeError("Install requirements.txt in the Pi virtual environment first") from error
        if ultralytics.__version__ != "8.4.159":
            raise RuntimeError("This package requires ultralytics==8.4.159; run pip install -r requirements.txt")

        for name in ("metadata.yaml", "model.ncnn.param", "model.ncnn.bin"):
            if not (model_path / name).is_file():
                raise RuntimeError(f"Missing NCNN file: {model_path / name}")
        metadata = yaml.safe_load((model_path / "metadata.yaml").read_text(encoding="utf-8"))
        manifest_path = model_path / "classes.json"
        manifest = (json.loads(manifest_path.read_text(encoding="utf-8"))
                    if manifest_path.is_file() else None)
        self.model_to_wire, self.class_names = model_class_mapping(metadata, manifest)

        torch.set_num_threads(threads)
        cv2.setNumThreads(1)
        self.model = YOLO(str(model_path), task="detect")
        self.tracker = tracker
        self.tracker_config = SCRIPT_DIRECTORY / "trackers" / f"{tracker}.yaml"
        if tracker != "none" and not self.tracker_config.is_file():
            raise RuntimeError(f"Missing tracker configuration: {self.tracker_config}")
        self.confidence = round(confidence * 1000)
        self.state = (TrackingState(track_retention, track_max_gap)
                      if tracker != "none" else None)
        self.reacquire_enabled = reacquire
        self.options = dict(imgsz=320, rect=False,
                            conf=confidence if tracker == "none" else candidate_confidence,
                            iou=iou, max_det=candidate_limit, device="cpu", verbose=False)
        LOG.info("Warm up %s (fixed 320x320 NCNN, %d classes mapped to COCO wire IDs)",
                 model_path, len(self.model_to_wire))
        self.model.predict(source=np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8),
                           **self.options)
        LOG.info("Tracker=%s candidate_limit=%d conf=%.2f; actual-observation expiry %.2fs, gap reset %.2fs",
                 tracker, candidate_limit, self.options["conf"], track_retention, track_max_gap)

    @property
    def tracker_session(self):
        return self.state.session if self.state else 0

    def _trackers(self):
        return getattr(getattr(self.model, "predictor", None), "trackers", [])

    def _reset_trackers(self):
        for tracker in self._trackers():
            tracker.reset()

    def class_name(self, class_id: int) -> str:
        return self.class_names[class_id]

    def detect(self, image, frame: VideoFrame | None = None, lock=None,
               preferred_class=None) -> list[DetectionBox]:
        if self.state:
            if frame is None:
                raise ValueError("tracked detection requires the sampled video frame")
            if self.state.prepare_frame(frame):
                self._reset_trackers()
                LOG.warning("Tracking session reset: %s session=%08x",
                            self.state.reset_reason, self.state.session)
            evict_track_ids(self._trackers(), self.state.expired_ids(frame.completed_at))
            results = self.model.track(source=image, persist=True,
                                       tracker=str(self.tracker_config), **self.options)
        else:
            results = self.model.predict(source=image, **self.options)
        if not results or results[0].boxes is None:
            return self._reply_detections([], frame, lock, preferred_class)
        result_boxes = results[0].boxes.cpu()
        xyxy = result_boxes.xyxy.numpy()
        scores = result_boxes.conf.numpy()
        classes = result_boxes.cls.numpy()
        ids = result_boxes.id.numpy() if self.state and result_boxes.id is not None else None
        observed_ids = set()
        if self.state:
            if ids is None:
                # Ultralytics leaves raw detections in Results when no confirmed
                # track exists. They must not acquire an invented tracking ID.
                return self._reply_detections([], frame, lock, preferred_class)
            for tracker in self._trackers():
                observed_ids.update(int(track.track_id) for track in tracker.tracked_stracks
                                    if track.is_activated and track.frame_id == tracker.frame_id)
        detections = []
        # Keep the full candidate set through association; apply the eight-slot
        # limit only after reserving the locked object's slot.
        for index in sorted(range(len(scores)), key=lambda i: float(scores[i]), reverse=True):
            class_id = wire_class_id(classes[index], self.model_to_wire)
            if class_id is None:
                continue
            box = box_from_xyxy(xyxy[index], float(scores[index]), class_id=class_id)
            if box is not None:
                if self.state:
                    numeric_id = float(ids[index])
                    if not math.isfinite(numeric_id) or not numeric_id.is_integer():
                        self.state.new_session("malformed-track-id")
                        self._reset_trackers()
                        return []
                    track_id = int(numeric_id)
                    if not 0 < track_id <= MAX_EXACT_TRACK_ID:
                        self.state.new_session("track-id-precision-limit")
                        self._reset_trackers()
                        return []
                    if track_id not in observed_ids:
                        continue
                    box = replace(box, track_id=track_id, flags=OBSERVED)
                detections.append(box)
            if not self.state and len(detections) == MAX_BOXES:
                break
        return self._reply_detections(detections, frame, lock, preferred_class)

    def _reply_detections(self, detections, frame, lock, preferred_class=None):
        # Empty frames also advance loss confirmation and clear a withdrawn lock.
        if self.state:
            detections, retired, must_reset = self.state.accept_observations(detections, frame.completed_at)
            if must_reset:
                self._reset_trackers()
                LOG.warning("Tracking session reset: %s session=%08x",
                            self.state.reset_reason, self.state.session)
                return []
            evict_track_ids(self._trackers(), retired)
            if callable(lock):
                lock = lock()  # honor a lock received while inference was running
            if self.reacquire_enabled:
                previous = self.state.active_alias
                detections = self.state.reacquire(detections, lock, frame.completed_at,
                                                  strong_confidence=self.confidence)
                current = self.state.active_alias
                if current is not None and current != previous:
                    LOG.info("Locked ID %d re-acquired through tracker ID %d",
                             current[1], current[0])
                elif current is None and previous is not None:
                    LOG.info("Locked ID %d observed again; dropped alias for tracker ID %d",
                             previous[1], previous[0])
            return choose_reply_boxes(detections, self.state.session, lock,
                                      strong_confidence=self.confidence,
                                      preferred_class=preferred_class)
        return detections


class YoloDisplayWorker(threading.Thread):
    """Keep YOLO overlays current without delaying a VLM control reply."""

    def __init__(self, detector, detector_lock, max_fps=5.0):
        super().__init__(name="yolo-display", daemon=True)
        if max_fps <= 0:
            raise ValueError("YOLO display FPS must be positive")
        self.detector = detector
        self.detector_lock = detector_lock
        self.min_interval = 1.0 / max_fps
        self.max_result_age = max(0.30, 2 * self.min_interval)
        self.condition = threading.Condition()
        self.pending = None
        self.result = None
        self.next_inference_at = 0.0
        self.enabled = False
        self.stopping = False
        self.failure = None
        self.start()

    def resume(self):
        with self.condition:
            self.enabled = True
            self.pending = None
            self.result = None
            self.next_inference_at = 0.0

    def pause(self):
        with self.condition:
            self.enabled = False
            self.pending = None
            self.result = None
        # A normal YOLO task must not race an older display inference.
        with self.detector_lock:
            pass

    def submit(self, image, frame):
        with self.condition:
            if (self.enabled and self.failure is None and
                    time.monotonic() >= self.next_inference_at):
                self.pending = (image.copy(), frame)
                self.condition.notify()

    def snapshot(self, now, sequence_epoch):
        with self.condition:
            if (not self.enabled or self.result is None or
                    self.result[1] != sequence_epoch or
                    now - self.result[0] > self.max_result_age):
                return []
            return self.result[2]

    def observation(self, now, sequence_epoch):
        with self.condition:
            if (not self.enabled or self.result is None or
                    self.result[1] != sequence_epoch or
                    now - self.result[0] > self.max_result_age):
                return None
            captured_at, _, boxes, image, frame_id, tracker_session = self.result
            return YoloObservation(image, tuple(boxes), frame_id, captured_at,
                                   tracker_session)

    def run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.pending is not None or self.stopping)
                if self.stopping:
                    return
                image, frame = self.pending
                self.pending = None
            try:
                with self.detector_lock:
                    with self.condition:
                        if not self.enabled:
                            continue
                        # Start-time spacing limits real inference, even if a
                        # previous detection was still running at the deadline.
                        self.next_inference_at = time.monotonic() + self.min_interval
                    boxes = self.detector.detect(image, frame)
                with self.condition:
                    if self.enabled:
                        self.result = (frame.first_packet_at, frame.sequence_epoch,
                                       boxes, image, getattr(frame, "frame_id", 0),
                                       getattr(self.detector, "tracker_session", 0))
            except Exception as error:
                with self.condition:
                    self.failure = error
                    self.enabled = False
                    self.pending = None
                    self.result = None
                return

    def stop(self):
        with self.condition:
            self.stopping = True
            self.pending = None
            self.condition.notify()
        self.join(timeout=1.0)


class FrameRateGate:
    """Sample camera frames at a target average rate without frame-period rounding loss."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.interval = None
        self.next_due = None

    def allow(self, now, interval):
        if interval != self.interval:
            self.interval = interval
            self.next_due = now
        if now < self.next_due:
            return False
        self.next_due += interval
        if self.next_due < now:
            # A slow inference or paused stream must not create catch-up bursts.
            self.next_due = now + interval
        return True


def merge_vlm_display_boxes(vlm_boxes, yolo_boxes):
    """Only the VLM target can be selected while its task owns the FBOX session."""
    if len(vlm_boxes) > 1:
        raise ValueError("a VLM task must publish at most one target")
    merged = list(vlm_boxes)
    for slot, box in enumerate(yolo_boxes[:MAX_BOXES - len(merged)], start=1):
        # The FPGA overlay draws flags=0 boxes, but its follow controller cannot
        # select them. High IDs cannot collide with the VLM target's 24-bit ID.
        merged.append(replace(box, track_id=0x80000000 | slot, flags=0))
    return merged


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen-ip", default="192.168.10.1", help="Pi Ethernet IPv4 address")
    parser.add_argument("--video-port", type=int, default=5000)
    parser.add_argument("--fpga-ip", default="192.168.10.2")
    parser.add_argument("--box-port", type=int, default=5001)
    parser.add_argument("--no-thermal", action="store_true",
                        help="Disable the MLX90640 temperature monitor")
    parser.add_argument("--thermal-library", type=Path,
                        help="Path to the locally built MLX90640 native library")
    parser.add_argument("--model", type=Path, default=SCRIPT_DIRECTORY / "best_ncnn_model")
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--tracker", choices=("botsort", "bytetrack", "none"), default="botsort",
                        help="BoT-SORT camera-motion compensation; none sends legacy detection-only v1")
    parser.add_argument("--candidate-confidence", type=float, default=0.1,
                        help="Internal tracking detections; weak observations cannot drive the car")
    parser.add_argument("--candidate-limit", type=int, default=64,
                        help="Candidates before tracking (>=32); wire reply stays limited to eight boxes")
    parser.add_argument("--track-retention", type=float, default=2.0,
                        help="Wall-clock lifetime of an unobserved ID, in seconds; keep it at "
                             "least the FPGA TARGET_RETAIN_MS (2000 ms in production)")
    parser.add_argument("--track-max-gap", type=float, default=0.6,
                        help="Reset tracker session if sampled inference frames are farther apart")
    parser.add_argument("--no-reacquire", action="store_true",
                        help="Do not relabel a replacement tracker ID back to the locked ID")
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--threads", type=int, default=4,
                        help="PyTorch CPU preprocessing/postprocessing threads")
    parser.add_argument("--frame-timeout", type=float, default=0.25,
                        help="Discard incomplete frames after this many seconds")
    parser.add_argument("--sequence-reset-timeout", type=float, default=2.0,
                        help="Reset IDs after stream idle, or old IDs with no accepted complete frame "
                             "for this many seconds")
    parser.add_argument("--max-frame-age", type=float, default=0.35,
                        help="Discard frames/results older than this from their first UDP line")
    parser.add_argument("--receive-buffer", type=int, default=4 * 1024 * 1024)
    parser.add_argument("--no-inference", action="store_true",
                        help="Receive and return zero-box ACKs without numpy/YOLO/OpenCV")
    parser.add_argument("--demo-box", action="store_true",
                        help="With --no-inference, return one fixed box to test the FPGA overlay")
    parser.add_argument("--preview", action="store_true", help="Open local OpenCV preview; q exits")
    parser.add_argument('--record', action='store_true', help='Save native camera video with measured boxes')
    parser.add_argument('--record-output', type=Path, help='New MKV path; default recordings/detections_timestamp.mkv')
    parser.add_argument('--record-fps', type=float, default=30.0, help='Saved CFR FPS; inference FPS is independent')
    parser.add_argument('--record-crf', type=int, default=18)
    parser.add_argument('--record-threads', type=int, default=2)
    parser.add_argument('--record-queue', type=int, default=4)
    parser.add_argument("--task-console", action="store_true",
                        help="Read Chinese motion, YOLO and VLM task phrases from stdin")
    parser.add_argument("--vlm-url", help="Local llama.cpp chat completions URL for LFM2.5-VL-450M")
    parser.add_argument("--task6-pc-url", help="Laptop cloud/YOLOE service, e.g. http://<PC-IP>:8765")
    parser.add_argument("--hsv-ball-config", type=Path,
                        help="Optional HSV ball thresholds JSON (Pi-local; no retraining)")
    parser.add_argument("--task6-mode", choices=("target", "yolo", "vision"), default="target",
                        help="target: DS+YOLO/HSV tasks; yolo: detector display; vision: DS search/approach, no HSV")
    parser.add_argument("--task6-hsv-display", action=argparse.BooleanOptionalAction, default=True,
                        help="Add Pi HSV balls in YOLO display mode; --no-task6-hsv-display skips HSV there (ball target tasks unchanged)")
    parser.add_argument("--task6-allow-mock", action="store_true",
                        help="Allow explicitly marked mock PC results; simulator use only")
    parser.add_argument("--task6-search-turn", action=argparse.BooleanOptionalAction, default=None,
                        help="Rotate/settle search (default on in target mode; --no-task6-search-turn disables)")
    parser.add_argument("--vlm-model", default="task6-vlm", help="llama.cpp --alias value")
    parser.add_argument("--vlm-only", action="store_true",
                        help="Skip YOLO inference and use only VLM visual tasks")
    parser.add_argument("--vlm-yolo-fps", type=float, default=8.0,
                        help="YOLO overlay and matched-target correction FPS during VLM tasks "
                             "(default: 8); VLM control replies are unaffected")
    parser.add_argument("--combined-yolo-fps", type=float, default=12.0,
                        help="Normal YOLO inference FPS when --vlm-url is also enabled "
                             "(default: 12); YOLO-only mode is unaffected")
    parser.add_argument("--combined-follow-fps", type=float, default=30.0,
                        help="YOLO inference FPS while an FPGA/command target is being followed "
                             "alongside VLM (default: 30)")
    parser.add_argument("--web-port", type=int,
                        help="Serve phone text/speech-input UI on Wi-Fi (e.g. 8088)")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="Exit after N box replies (0 = run continuously; useful for tests)")
    parser.add_argument("--log-interval", type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.task6_search_turn is None:
        args.task6_search_turn = bool(args.task6_pc_url and args.task6_mode in {"target", "vision"})
    if not (1 <= args.video_port <= 65535 and 1 <= args.box_port <= 65535):
        parser.error("UDP ports must be 1..65535")
    if not (0.35 <= args.confidence <= 1 and 0 < args.iou <= 1):
        parser.error("confidence must be >=0.35 and <=1; iou must be in (0,1]")
    if not 0 < args.candidate_confidence <= 0.1 or args.candidate_limit < 32:
        parser.error("candidate-confidence must be in (0,0.1]; candidate-limit must be >=32")
    if args.track_retention <= 0 or args.track_max_gap <= 0:
        parser.error("tracking lifetime/gap must be positive")
    if args.threads < 1 or args.max_frames < 0 or args.receive_buffer < 65536:
        parser.error("invalid threads, max-frames, or receive-buffer")
    if args.frame_timeout <= 0 or args.sequence_reset_timeout <= args.frame_timeout:
        parser.error("sequence-reset-timeout must exceed a positive frame-timeout")
    if (args.max_frame_age <= 0 or args.log_interval <= 0 or
            not math.isfinite(args.vlm_yolo_fps) or args.vlm_yolo_fps <= 0 or
            not math.isfinite(args.combined_yolo_fps) or args.combined_yolo_fps <= 0 or
            not math.isfinite(args.combined_follow_fps) or args.combined_follow_fps <= 0):
        parser.error("max-frame-age, log-interval, and YOLO FPS limits must be finite and positive")
    if args.demo_box and not args.no_inference:
        parser.error("--demo-box requires --no-inference")
    if args.vlm_only and not args.vlm_url:
        parser.error("--vlm-only requires --vlm-url")
    if args.vlm_url and args.no_inference:
        parser.error("--vlm-url cannot be combined with --no-inference")
    if args.task6_pc_url and (args.vlm_url or args.vlm_only or args.demo_box):
        parser.error("--task6-pc-url cannot be combined with legacy VLM/demo-box options")
    if (args.task6_allow_mock or args.task6_search_turn) and not args.task6_pc_url:
        parser.error("task6 options require --task6-pc-url")
    if args.task6_mode == "yolo" and (not args.task6_pc_url or args.task6_search_turn):
        parser.error("YOLO display requires --task6-pc-url and cannot use search-turn")
    if args.task6_mode == 'vision' and not args.task6_pc_url:
        parser.error('DS vision mode requires PC URL')
    if args.web_port is not None and not 1 <= args.web_port <= 65535:
        parser.error("--web-port must be 1..65535")
    if (not math.isfinite(args.record_fps) or not 0 < args.record_fps <= 120 or
            not 0 <= args.record_crf <= 51 or args.record_threads < 1 or not 1 <= args.record_queue <= 64):
        parser.error('invalid recording FPS/CRF/threads/queue')
    if args.record_output is not None and not args.record:
        parser.error('--record-output requires --record')
    return args


def run(args, stop_event=None) -> int:
    LOG.info("Task6 receiver revision=20261031-ds-search-r3 source=%s", Path(__file__).resolve())
    detector = None if args.no_inference or args.vlm_only or args.task6_pc_url else ObjectDetector(args.model.expanduser().resolve(),
                                                            args.confidence, args.iou, args.threads,
                                                            args.tracker, args.candidate_confidence,
                                                            args.candidate_limit, args.track_retention,
                                                            args.track_max_gap,
                                                            reacquire=not args.no_reacquire)
    cv2 = None
    if args.preview or args.record:
        try:
            import cv2
        except ImportError as error:
            raise RuntimeError("--preview/--record requires numpy and OpenCV") from error
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver = None
    recorder = None
    recording_error_reported = False
    task_queue = queue.Queue()
    controller = None
    manual_refresh_stop = threading.Event()
    manual_refresh_thread = None
    vlm = None
    thermal = None
    thermal_gate = None
    detector_lock = threading.Lock()
    display_worker = None
    display_worker_enabled = False
    remote_native_frames = OrderedDict()
    remote_camera_paused = False
    combined_yolo_interval = (1.0 / args.combined_yolo_fps
                              if args.vlm_url and detector is not None else 0.0)
    combined_follow_interval = (1.0 / args.combined_follow_fps
                                if args.vlm_url and detector is not None else 0.0)
    normal_yolo_gate = FrameRateGate()
    sent, stale, screen_frames = 0, 0, 0
    last_log = started = time.monotonic()
    last_sent = 0
    last_remote_log = started
    nonempty_replies = 0
    last_box_count = last_hsv_count = 0
    last_box_classes = "-"
    last_wire_session = 0
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, args.receive_buffer)
        sock.settimeout(0.05)
        sock.bind((args.listen_ip, args.video_port))
        latest = LatestFrame()
        locks = LockMailbox()
        receiver = UdpVideoReceiver(sock, latest, args.fpga_ip, args.frame_timeout,
                                    args.sequence_reset_timeout, locks)
        receiver.start()
        if args.record:
            from detection_recording import ObjectRecorder
            if args.task6_pc_url and args.record_output is None:
                filename = time.strftime("detections_%Y%m%d_%H%M%S_") + str(time.time_ns()) + ".mkv"
                args.record_output = SCRIPT_DIRECTORY / "recordings" / args.task6_mode / filename
            recorder = ObjectRecorder(args.record_output, fps=args.record_fps, crf=args.record_crf,
                                      threads=args.record_threads, queue_frames=args.record_queue)
            LOG.info('Recording camera boxes / HDMI pages -> %s (Ctrl+C saves)', recorder.output)
        if args.task_console or args.vlm_url or args.web_port or args.task6_pc_url:
            if detector is None and not (args.vlm_url or args.task6_pc_url):
                raise RuntimeError("task commands require visual inference")
            controller = TaskController(sock, (args.fpga_ip, args.box_port))
            def refresh_manual_motion():
                while not manual_refresh_stop.wait(0.05):
                    try:
                        controller.refresh_manual_motion()
                    except OSError:
                        LOG.exception("Manual motion refresh failed; FPGA timeout will stop the car")
                        return
            manual_refresh_thread = threading.Thread(target=refresh_manual_motion,
                                                     name="manual-motion-refresh", daemon=True)
            manual_refresh_thread.start()
        if args.task6_pc_url:
            sys.path.insert(0, str(SCRIPT_DIRECTORY.parent))
            if args.task6_mode == 'vision':
                from task6.pi_client import RemoteTaskController
                vlm = RemoteTaskController(controller.send, args.task6_pc_url,
                    max_age=args.max_frame_age, allow_mock=args.task6_allow_mock,
                    search_turn=args.task6_search_turn, mode='vision')
            else:
                from task6.hybrid_pi import HybridTaskController
                vlm = HybridTaskController(controller.send, args.task6_pc_url,
                    max_age=args.max_frame_age, allow_mock=args.task6_allow_mock,
                    search_turn=args.task6_search_turn, mode=args.task6_mode,
                    hsv_config=args.hsv_ball_config, hsv_display=args.task6_hsv_display)
            if args.task6_mode == 'vision':
                LOG.info('Task6 DeepSeek direct image grounding; no YOLO/HSV; explicit approach instructions supported')
                LOG.info('DS confirmed-miss search turns %s; pause/settle views discard old cloud answers',
                         'enabled' if args.task6_search_turn else 'disabled')
            elif args.task6_mode == "yolo":
                vlm.start_detection()
            LOG.info("Remote display mode: %s", args.task6_mode)
            LOG.info("Task6 PC communication auto-reconnect enabled; stop and reconfirm target after reconnect")
            LOG.info("Task6 observation-age STOP/reselect disabled; stale-frame rejection and FPGA protections unchanged")
            if args.task6_mode == "yolo":
                LOG.info("Task6 PC YOLO display; Pi-local HSV ball detection %s",
                         "enabled" if args.task6_hsv_display else "disabled")
            elif args.task6_mode == 'target':
                LOG.info("Task6 PC cloud/YOLOE/BoT-SORT + Pi-local HSV ball target tasks enabled")
        if args.vlm_url:
            # YOLO remains available for its own commands and HDMI overlay, but
            # a VLM task is never gated or steered by a second detector.
            vlm = VlmTaskController(controller.send, VlmClient(args.vlm_url, args.vlm_model))
            if detector is not None:
                display_worker = YoloDisplayWorker(detector, detector_lock, args.vlm_yolo_fps)
                LOG.info("YOLO display boxes enabled during VLM tasks at up to %.1f FPS (display only)",
                         args.vlm_yolo_fps)
                LOG.info("YOLO inference while VLM is enabled: %.1f FPS idle, %.1f FPS following",
                         args.combined_yolo_fps, args.combined_follow_fps)
        if not args.no_thermal:
            stop_preview_service(quiet=True)
            thermal_gate = ThermalTaskGate(lambda: vlm, (args.fpga_ip, args.box_port))
            thermal = ThermalMonitor(sock, (args.fpga_ip, args.box_port),
                                     task_active=thermal_gate,
                                     library_path=args.thermal_library)
            thermal.start()
        if args.web_port:
            from web_control import start_web_control
            def web_status():
                lock = locks.snapshot()
                if controller.manual_opcode is not None:
                    name = next((name for name, code in MOTION.items()
                                 if code == controller.manual_opcode), "手动运动")
                    status = {"phase": "manual", "target": name, "bbox": None,
                              "message": "方向指令持续有效；点击“立即停车”可停止"}
                else:
                    status = (vlm.status() if vlm is not None else
                              {"phase": controller.phase, "target": "", "message": ""})
                status["fpga_lock"] = (f"ID {lock.track_id} / 会话 {lock.tracker_session:08x}"
                                       if lock is not None else "未锁定")
                status['thermal'] = thermal.snapshot() if thermal is not None else {"enabled": False}
                last_frame = receiver.last_frame_at
                status['communication_state'] = ('正常' if last_frame is not None and
                    time.monotonic()-last_frame <= 2.0 else '等待画面')
                return status
            access_token = start_web_control(task_queue, web_status,
                                             args.web_port, allow_vlm=vlm is not None and args.task6_mode != "yolo",
                                             allow_yolo=detector is not None,
                                             thermal_setter=thermal.set_threshold if thermal is not None else None,
                                             thermal_frames=thermal.frame_snapshot if thermal is not None else None,
                                             thermal_manual=thermal.set_manual if thermal is not None else None)
            LOG.info("Phone UI: http://<Pi-Wi-Fi-IP>:%d/%s", args.web_port, access_token)
        if args.task_console:
            def read_commands():
                while True:
                    try:
                        task_queue.put(input("任务指令> "))
                    except EOFError:
                        return
            threading.Thread(target=read_commands, name="task-console", daemon=True).start()
        actual_buffer = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        LOG.info("READY %s:%d -> %s:%d, FVID camera / FREC HDMI; detection view640x360; SO_RCVBUF=%d",
                 args.listen_ip, args.video_port, args.fpga_ip, args.box_port, actual_buffer)
        if args.no_inference:
            LOG.info("Network test mode: %s", "fixed test box" if args.demo_box else "zero boxes")
        while (not args.max_frames or sent + screen_frames < args.max_frames) and not (stop_event and stop_event.is_set()):
            if recorder is not None and recorder.error and not recording_error_reported:
                LOG.error('Recording failed; recognition continues: %s', recorder.error)
                recording_error_reported = True
            while controller is not None:
                try:
                    command = task_queue.get_nowait()
                except queue.Empty:
                    break
                if thermal is not None:
                    thermal.reset_task()
                intent = parse_text(command)
                if args.task6_pc_url and args.task6_mode == "yolo":
                    if intent is not None and intent[0] == "motion":
                        controller.submit(command)
                    else:
                        LOG.warning("YOLO-only shows detector boxes; use target mode for search instructions")
                    continue
                goal = parse_visual_goal(command) if vlm is not None else None
                if args.task6_pc_url and intent is not None and intent[0] == "target" and goal is not None:
                    intent = None  # Search phrases use the new semantic pipeline.
                if intent is not None:
                    if intent[0] == "target" and detector is None:
                        LOG.warning("YOLO target commands require YOLO inference")
                        continue
                    if vlm is not None:
                        vlm.cancel()
                    controller.submit(command)
                elif goal is not None:
                    controller.submit("停车")
                    vlm.submit(goal)
                else:
                    LOG.warning("Unknown command: %s", command)
            if vlm is not None:
                vlm.tick()
                vlm.observe_lock(locks.snapshot())
            if receiver.failure is not None:
                raise RuntimeError("UDP receive thread terminated") from receiver.failure
            if display_worker is not None and display_worker.failure is not None:
                raise RuntimeError("YOLO display worker terminated") from display_worker.failure
            # Log even if no processed PC result arrives. The old block below
            # could be skipped forever by the no-result 'continue' path.
            diagnostics_now = time.monotonic()
            if args.task6_pc_url and diagnostics_now-last_remote_log >= args.log_interval:
                d = vlm.diagnostics()
                LOG.info("Task6 mode=%s backend=%s active=%s phase=%s pc_state=%s pc_sequence=%s "
                         "pc_boxes=%s inference_ms=%s last_boxes=%d hsv=%d classes=%s "
                         "nonempty_replies=%d reason=%s",
                         d['mode'], d.get('backend', 'PC-YOLO'), d['active'], d['phase'], d['pc_state'], d['sequence'],
                         d['pc_boxes'], d['inference_ms'], last_box_count, last_hsv_count,
                         last_box_classes, nonempty_replies, d['error'])
                last_remote_log = diagnostics_now
            frame = latest.take(timeout=0.1)
            now = time.monotonic()
            if frame is not None:
                if now - frame.first_packet_at > args.max_frame_age:
                    stale += 1
                    frame = None
            if frame is not None and frame.source == "hdmi":
                # The existing FPGA switches from FVID to final HDMI pixels.
                # These are for recording only, never camera observations.
                if args.task6_pc_url and vlm.active:
                    if args.task6_mode == "yolo":
                        vlm.cancel()
                        remote_camera_paused = True
                    else:
                        vlm.abort("HDMI page stream selected; return to camera and resubmit task")
                    remote_native_frames.clear()
                if display_worker is not None and display_worker_enabled:
                    display_worker.pause()
                    display_worker_enabled = False
                normal_yolo_gate.reset()
                if args.preview or (recorder is not None and not recorder.error):
                    page_image = rgb565_to_bgr(frame)
                    if recorder is not None and not recorder.error:
                        recorder.submit(frame, page_image, [])
                    if args.preview:
                        cv2.imshow("FPGA UDP / Pi objects", page_image)
                screen_frames += 1
                frame = None
            if frame is not None:
                if args.task6_pc_url and args.task6_mode == "yolo" and remote_camera_paused:
                    vlm.start_detection()
                    remote_camera_paused = False
                inference_start = now
                using_vlm = vlm is not None and vlm.active
                mixed_vlm_frame = using_vlm and frame.version != 1
                if display_worker is not None and mixed_vlm_frame != display_worker_enabled:
                    if mixed_vlm_frame:
                        display_worker.resume()
                    else:
                        display_worker.pause()
                    display_worker_enabled = mixed_vlm_frame
                if using_vlm:
                    normal_yolo_gate.reset()
                elif combined_yolo_interval:
                    following = (locks.snapshot() is not None or
                                 (controller is not None and controller.phase == "following"))
                    interval = combined_follow_interval if following else combined_yolo_interval
                    if not normal_yolo_gate.allow(now, interval):
                        # Do not republish an old detection as a fresh observation.
                        continue
                native_image = rgb565_to_bgr(frame) if detector is not None or args.preview or using_vlm or args.record else None
                image = native_image
                if native_image is not None and (frame.width, frame.height) != (VIDEO_WIDTH, VIDEO_HEIGHT):
                    from detection_recording import recognition_view
                    image = recognition_view(native_image)
                if args.task6_pc_url and using_vlm:
                    if frame.version == 1:
                        vlm.abort("Task6 requires FVID v2/v3 tracking firmware")
                        continue
                    if args.record or args.preview:
                        remote_native_frames[frame.frame_id] = (frame, native_image)
                        while len(remote_native_frames) > 16:
                            remote_native_frames.popitem(last=False)
                    boxes = vlm.process(image, frame.frame_id, captured_at=frame.first_packet_at,
                                        sequence_epoch=frame.sequence_epoch, detail_image=native_image)
                    if boxes is None:
                        # No NEW processed result yet. Never relabel the last box
                        # with this newer camera frame's identity/freshness.
                        continue
                elif mixed_vlm_frame:
                    yolo_observation = (display_worker.observation(time.monotonic(), frame.sequence_epoch)
                                        if display_worker is not None else None)
                    vlm_boxes = vlm.process(image, frame.frame_id, yolo_observation)
                    if display_worker is not None:
                        display_worker.submit(image, frame)
                        yolo_boxes = display_worker.snapshot(time.monotonic(), frame.sequence_epoch)
                        boxes = merge_vlm_display_boxes(vlm_boxes, yolo_boxes)
                    else:
                        boxes = vlm_boxes
                elif detector is not None:
                    preferred = controller.target_class if controller is not None and controller.phase in ("search", "select") else None
                    with detector_lock:
                        if preferred is None:
                            boxes = detector.detect(image, frame, locks.snapshot)
                        else:
                            boxes = detector.detect(image, frame, locks.snapshot,
                                                    preferred_class=preferred)
                elif args.demo_box:
                    boxes = [DetectionBox(160, 90, 480, 270, 1000, 0)]
                else:
                    boxes = []
                reply_capture = (vlm.reply_captured_at if args.task6_pc_url and using_vlm
                                 else frame.first_packet_at)
                reply_frame = (vlm.reply_frame_id if args.task6_pc_url and using_vlm else frame.frame_id)
                if time.monotonic() - reply_capture > args.max_frame_age:
                    stale += 1
                else:
                    if controller is not None and frame.version != 1 and not using_vlm:
                        boxes = controller.prepare_boxes(boxes, frame.frame_id)
                    # Same socket guarantees src port 5000; the RTL filters this port.
                    tracker_session = vlm.session if using_vlm else detector.tracker_session if detector else 0
                    if frame.version == 1:
                        # A v1 FPGA has no tracking/control fields. Keep its
                        # existing display protocol operational during upgrades.
                        tracker_session = 0
                        boxes = [replace(box, track_id=0, flags=0) for box in boxes]
                    wire_label=vlm.english_label if using_vlm and tracker_session else ""
                    sock.sendto(encode_boxes(reply_frame, boxes, tracker_session=tracker_session,label=wire_label),
                                (args.fpga_ip, args.box_port))
                    last_wire_session = tracker_session
                    last_box_count = len(boxes)
                    last_hsv_count = sum(80 <= b.class_id <= 83 for b in boxes)
                    last_box_classes = ','.join(str(b.class_id) for b in boxes) or '-'
                    nonempty_replies += int(bool(boxes))
                    if using_vlm and frame.version != 1:
                        vlm.after_sent()
                    elif controller is not None and frame.version != 1:
                        controller.after_boxes_sent(frame.frame_id)
                    sent += 1
                    if args.preview or (recorder is not None and not recorder.error):
                        from detection_recording import draw_detections
                        def label_for_box(box):
                            from hsv_balls import BALL_LABELS
                            if box.class_id in BALL_LABELS:
                                return BALL_LABELS[box.class_id]
                            if args.task6_pc_url and args.task6_mode == "yolo":
                                return COCO_NAMES[box.class_id]
                            return (vlm.english_label if using_vlm and box.class_id == VLM_WIRE_CLASS and vlm.goal
                                    else detector.class_name(box.class_id) if detector is not None else 'person')
                        render_frame, render_image = (remote_native_frames.get(reply_frame, (None, None))
                            if args.task6_pc_url and using_vlm else (frame, native_image))
                        if render_image is None:
                            continue  # Never overlay an old result on a different camera frame.
                        recording_lock = locks.snapshot()
                        recording_selected_id = (recording_lock.track_id
                            if args.task6_pc_url and recording_lock is not None and
                            recording_lock.tracker_session == tracker_session else 0)
                        annotated, annotations = draw_detections(render_image, boxes, label_for_box,
                            selected_track_id=recording_selected_id)
                        if using_vlm and vlm.goal:
                            for annotation in annotations:
                                annotation['goal_description']=vlm.goal.description
                        if recorder is not None and not recorder.error:
                            recorder.submit(render_frame, annotated, annotations)
                        if args.preview:
                            cv2.imshow("FPGA UDP / Pi objects", annotated)
                LOG.debug("frame=%d objects=%d processing_ms=%.1f", frame.frame_id, len(boxes),
                          (time.monotonic() - inference_start) * 1000)
            if args.preview and cv2.waitKey(1) & 0xFF == ord("q"):
                break
            now = time.monotonic()
            if now - last_log >= args.log_interval:
                stats = dict(receiver.assembler.stats)
                LOG.info("replies=%d fps=%.1f complete=%d expired=%d malformed=%d duplicate=%d "
                         "overwritten=%d stale=%d sequence_resets=%d rejected_sources=%d session=%08x "
                         "resets=%d reacquired=%d",
                         sent, (sent - last_sent) / (now - last_log), stats["complete"],
                         stats["expired"], stats["malformed"], stats["duplicate"],
                         latest.replaced, stale, stats["sequence_resets"], receiver.rejected_sources,
                         last_wire_session,
                         detector.state.reset_count if detector and detector.state else 0,
                         detector.state.reacquire_count if detector and detector.state else 0)
                last_log, last_sent = now, sent
                LOG.info("hdmi_complete=%d hdmi_recorded=%d hdmi_malformed=%d",
                         receiver.screen_assembler.stats["complete"], screen_frames,
                         receiver.screen_assembler.stats["malformed"])
                if recorder is not None:
                    LOG.info('recorded=%d repeat=%d record_queue_drop=%d input=%s source=%s', recorder.writer.written,
                             recorder.writer.repeated, recorder.writer.queue_drops,
                             receiver.screen_assembler.geometry if receiver.last_source == "hdmi" else receiver.assembler.geometry,
                             receiver.last_source)
    except KeyboardInterrupt:
        LOG.info("Stopped by user")
    finally:
        manual_refresh_stop.set()
        if manual_refresh_thread is not None:
            manual_refresh_thread.join(timeout=1.0)
        if controller is not None:
            try:
                controller.submit("停车")
            except OSError:
                pass
        if thermal is not None:
            thermal.stop()
        if thermal_gate is not None:
            thermal_gate.close()
        if display_worker is not None:
            display_worker.stop()
        if args.task6_pc_url and vlm is not None:
            vlm.close()
        if receiver is not None:
            receiver.stop()
        sock.close()
        if recorder is not None:
            recorder.close()
            if recorder.error:
                LOG.error('Recording failed: %s', recorder.error)
            elif recorder.writer.written:
                LOG.info('SAVED annotated video %s (%d frames)', recorder.output, recorder.writer.written)
            else:
                LOG.warning('No annotated frame recorded; check firmware, SW4 and incoming video')
        if cv2 is not None and args.preview:
            cv2.destroyAllWindows()
        LOG.info("Stopped: %d replies in %.1fs, %d stale frames/results", sent,
                 time.monotonic() - started, stale)
    return 0


def main(argv=None):
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    args = parse_args(argv)
    try:
        return run(args)
    except (RuntimeError, OSError) as error:
        LOG.error("%s", error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
