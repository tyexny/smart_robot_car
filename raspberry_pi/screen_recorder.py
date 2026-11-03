#!/usr/bin/env python3
"""Receive FPGA FREC screen rows and record a wall-clock-timed MKV using FFmpeg.

No YOLO, numpy or OpenCV dependency. Native 1280x720 FREC v3 segmented
RGB565BE, older RGB332 and legacy 640x360 RGB565BE frames are accepted.
RGB332 is expanded to its palette without spatial resampling. Frames
enter the bounded encoder queue. Missing frames repeat the last good frame;
the JSONL sidecar records repeats, receive times and original FPGA frame IDs.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import queue
import shutil
import socket
import subprocess
import threading
import time
from datetime import datetime

from protocol import (FrameAssembler, VideoLine, VIDEO_HEADER, VIDEO_WIDTH, VIDEO_HEIGHT,
                      decode_video_line, ProtocolError)

SCREEN_MAGIC = b"FREC"
SCREEN_WIDTH, SCREEN_HEIGHT = 1280, 720


def _palette565(value):
    r, g, b = value >> 5, (value >> 2) & 7, value & 3
    return (((r << 2) | (r >> 1)) << 11) | (((g << 3) | g) << 5) | ((b << 3) | (b << 1) | (b >> 1))


_RGB332_HI = bytes(_palette565(v) >> 8 for v in range(256))
_RGB332_LO = bytes(_palette565(v) & 255 for v in range(256))


def expand_rgb332(pixels):
    # C-level translations and strided copies; no per-pixel Python loop.
    result = bytearray(len(pixels) * 2)
    result[0::2] = pixels.translate(_RGB332_HI)
    result[1::2] = pixels.translate(_RGB332_LO)
    return bytes(result)


class ScreenAssembler(FrameAssembler):
    """Accept screen packets only; camera FVID/audio cannot become screen video."""

    def __init__(self, **kwargs):
        self._auto_geometry = "width" not in kwargs and "height" not in kwargs
        if self._auto_geometry:
            kwargs.update(width=SCREEN_WIDTH, height=SCREEN_HEIGHT)
        super().__init__(**kwargs)
        self.ignored_packets = 0
        self.old_video_packets = 0
        self.wire_format = None
        self.wire_version = None

    def expire(self, now=None):
        super().expire(time.perf_counter() if now is None else now)

    def feed(self, packet, now=None):
        # Explicit high-resolution monotonic clock in BOTH assembler and
        # writer. Some Windows Python versions have a coarse monotonic().
        now = time.perf_counter() if now is None else now
        if packet[:4] != SCREEN_MAGIC:
            self.ignored_packets += 1
            self.old_video_packets += int(packet[:4] == b"FVID")
            self.expire(now)
            return None
        self.stats["packets"] += 1
        if len(packet) >= 8 and packet[4] == 3:
            # Same split-row RGB565 layout as FVID, with a separate screen
            # assembler. HDMI page packets cannot update camera lock feedback.
            try:
                line = decode_video_line(b"FVID" + packet[4:], *(self.geometry or (None, None)))
                if line.lock_session or line.lock_id or line.lock_flags:
                    raise ProtocolError("screen packet carries camera lock fields")
                if self.wire_version not in (None, 3) or self.wire_format not in (None, 1):
                    raise ProtocolError("screen wire profile changed")
            except ProtocolError:
                self.stats["malformed"] += 1
                self.expire(now)
                return None
            self.wire_version, self.wire_format = 3, 1
            return self.feed_line(line, now, count_packet=False)
        if len(packet) < VIDEO_HEADER.size:
            self.stats["malformed"] += 1
            self.expire(now)
            return None
        _, version, fmt, header, fid, width, height, row, count, reserved = VIDEO_HEADER.unpack_from(packet)
        valid = (version == 1 and self.wire_version in (None, 1) and header == VIDEO_HEADER.size and fmt in (1,2)
                 and count == 1 and reserved == 0 and row < height
                 and len(packet) == header + width * (2 if fmt == 1 else 1))
        if self._auto_geometry and self.wire_format is None:
            valid = valid and (width,height,fmt) in ((640,360,1),(1280,720,2))
        else:
            valid = valid and (width,height) == (self.width,self.height)
        valid = valid and (self.wire_format is None or self.wire_format == fmt)
        if not valid:
            self.stats["malformed"] += 1
            self.expire(now)
            return None
        if self._auto_geometry and self.wire_format is None:
            self.width,self.height = width,height
        self.wire_format = fmt
        self.wire_version = 1
        pixels = packet[header:]
        if fmt == 2:
            pixels = expand_rgb332(pixels)
        return self.feed_line(VideoLine(fid,width,height,row,pixels),now,count_packet=False)


def encoder_command(ffmpeg, output, fps, crf, threads, width=VIDEO_WIDTH, height=VIDEO_HEIGHT,
                    pixel_format="rgb565be"):
    return [ffmpeg, "-hide_banner", "-loglevel", "warning", "-nostdin", "-n",
            "-f", "rawvideo", "-pixel_format", pixel_format, "-video_size",
            f"{width}x{height}", "-framerate", str(fps), "-i", "pipe:0",
            "-an", "-c:v", "libx264", "-preset", "ultrafast", "-crf", str(crf),
            "-threads", str(threads), "-pix_fmt", "yuv420p", "-f", "matroska", str(output)]


class ScreenWriter(threading.Thread):
    """Encoder stalls cannot block the UDP receive loop or grow memory forever."""

    def __init__(self, output, *, fps=30.0, crf=18, threads=3, queue_frames=16,
                 ffmpeg="ffmpeg", stop_event=None, pixel_format="rgb565be",
                 input_label="FREC HDMI, decoded RGB565BE (v3 segmented / v1 legacy)", frame_metadata=None):
        super().__init__(name="screen-encoder", daemon=True)
        self.output = Path(output)
        self.sidecar = self.output.with_suffix(self.output.suffix + ".jsonl")
        self.encoder_log = self.output.with_suffix(self.output.suffix + ".ffmpeg.log")
        self.fps, self.crf, self.threads = fps, crf, threads
        self.ffmpeg = ffmpeg
        self.pixel_format, self.input_label = pixel_format, input_label
        self.frame_metadata = frame_metadata
        self.frames = queue.Queue(maxsize=queue_frames)
        self.stop_event = stop_event if stop_event is not None else threading.Event()
        self.error = None
        self.written = self.repeated = self.queue_drops = self.superseded = 0
        self.epoch = None
        self.last_encoded_at = None

    def submit(self, frame):
        try:
            self.frames.put_nowait(frame)
        except queue.Full:
            # Keep latency bounded. The sidecar reports this separate from
            # missing UDP rows; CFR output retains elapsed recording time.
            try:
                self.frames.get_nowait()
            except queue.Empty:
                pass
            self.queue_drops += 1
            self.frames.put_nowait(frame)

    def run(self):
        process = None
        try:
            with self.encoder_log.open("x", encoding="utf-8") as encoder_log, \
                    self.sidecar.open("x", encoding="utf-8") as metadata:
                last = None
                while not self.stop_event.is_set() and last is None:
                    try:
                        last = self.frames.get(timeout=0.1)
                    except queue.Empty:
                        pass
                if last is None:
                    return
                geometry = (last.width,last.height)
                metadata.write(json.dumps({"event":"session", "started":datetime.now().astimezone().isoformat(),
                    "width":last.width,"height":last.height,"fps":self.fps,
                    "input":self.input_label,"audio":False})+"\n")
                # Anchor at the first complete frame, before FFmpeg startup.
                # While catching up, never use a frame received AFTER its
                # output tick: that would erase the start of the recording.
                self.epoch = last.completed_at
                process = subprocess.Popen(encoder_command(self.ffmpeg, self.output, self.fps,
                                                            self.crf, self.threads, *geometry,
                                                            pixel_format=self.pixel_format),
                                           stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                           stderr=encoder_log,
                                           start_new_session=os.name != "nt")
                next_tick = self.epoch
                previous_key = None
                pending = None
                while not self.stop_event.is_set():
                    if self.stop_event.wait(max(0, next_tick - time.perf_counter())):
                        break
                    # Newest complete frame available AT this output tick wins.
                    drained = 0
                    while True:
                        if pending is None:
                            try:
                                pending = self.frames.get_nowait()
                            except queue.Empty:
                                break
                        if pending.completed_at > next_tick:
                            break
                        last = pending
                        pending = None
                        drained += 1
                    self.superseded += max(0, drained - 1)
                    key = (getattr(last,'source',None), last.sequence_epoch, last.frame_id)
                    if (last.width,last.height) != geometry:
                        raise ValueError("input geometry changed; restart recording after changing firmware")
                    repeated = key == previous_key
                    process.stdin.write(last.pixels)
                    event = {"event": "frame", "video_frame": self.written,
                        "video_seconds": round(self.written / self.fps, 6),
                        "source_frame_id": last.frame_id, "sequence_epoch": last.sequence_epoch,
                        "source_received_seconds": round(last.completed_at - self.epoch, 6),
                        "written_seconds": round(time.perf_counter() - self.epoch, 6),
                        "repeated": repeated, "queue_drops": self.queue_drops}
                    if self.frame_metadata is not None:
                        event.update(self.frame_metadata(last))
                    metadata.write(json.dumps(event, ensure_ascii=False) + "\n")
                    self.written += 1
                    self.last_encoded_at = last.completed_at
                    self.repeated += int(repeated)
                    previous_key = key
                    next_tick = self.epoch + self.written / self.fps
                    if self.written % max(1, round(self.fps)) == 0:
                        metadata.flush()
                metadata.write(json.dumps({"event": "end", "frames": self.written,
                                            "repeated": self.repeated, "queue_drops": self.queue_drops,
                                            "superseded": self.superseded}) + "\n")
        except Exception as exc:
            self.error = str(exc)
            self.stop_event.set()
        finally:
            if process is not None:
                try:
                    process.stdin.close()
                except OSError:
                    pass
                try:
                    code = process.wait(timeout=30)
                    if code:
                        self.error = self.error or f"FFmpeg exited {code}; see {self.encoder_log}"
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                    self.error = self.error or "FFmpeg did not finish within 30 seconds"


def positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen", default="192.168.10.1")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--fps", type=positive_float, default=30.0)
    parser.add_argument("--duration", type=positive_float, help="Stop after this many seconds, including wait for first frame")
    parser.add_argument("--crf", type=int, default=18)
    parser.add_argument("--threads", type=int, default=3)
    parser.add_argument("--queue-frames", type=int, default=16)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not 0 <= args.crf <= 51 or args.threads < 1 or args.queue_frames < 1:
        parser.error("invalid port, crf, threads or queue size")
    ffmpeg = shutil.which(args.ffmpeg)
    if ffmpeg is None:
        parser.error("FFmpeg not found; install with: sudo apt install ffmpeg")
    output = args.output or Path("recordings") / datetime.now().strftime("mapping_%Y%m%d_%H%M%S.mkv")
    if output.suffix.lower() != ".mkv":
        parser.error("output must end in .mkv")
    output.parent.mkdir(parents=True, exist_ok=True)
    for path in (output, output.with_suffix(output.suffix + ".jsonl"), output.with_suffix(output.suffix + ".ffmpeg.log")):
        if path.exists():
            parser.error(f"refusing to overwrite {path}")
    stop = threading.Event()
    assembler = ScreenAssembler()
    writer = ScreenWriter(output, fps=args.fps, crf=args.crf, threads=args.threads,
                          queue_frames=args.queue_frames, ffmpeg=ffmpeg, stop_event=stop)
    started = time.perf_counter()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            sock.bind((args.listen, args.port))
            sock.settimeout(0.2)
            writer.start()
            print(f"READY {args.listen}:{args.port} -> {output}; turn SW4 ON, select map/navigation page", flush=True)
            print("No YOLO or FBOX replies. Ctrl+C stops and finalizes MKV.", flush=True)
            next_status = started + 5
            while not stop.is_set():
                now = time.perf_counter()
                if args.duration and now - started >= args.duration:
                    break
                try:
                    packet, _ = sock.recvfrom(2048)
                except socket.timeout:
                    assembler.expire(time.perf_counter())
                else:
                    frame = assembler.feed(packet)
                    if frame is not None:
                        writer.submit(frame)
                if now >= next_status:
                    print(f"complete={assembler.stats['complete']} expired={assembler.stats['expired']} "
                          f"malformed={assembler.stats['malformed']} encoded={writer.written} "
                          f"repeat={writer.repeated} queue_drop={writer.queue_drops} "
                          f"frec={assembler.stats['packets']} old_video={assembler.old_video_packets} "
                          f"input={assembler.width}x{assembler.height}", flush=True)
                    next_status = now + 5
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        print(f"ERROR: {exc}", flush=True)
        return 1
    finally:
        stop.set()
        if writer.ident is not None:
            writer.join(timeout=35)
    if writer.is_alive() or writer.error:
        print(f"ERROR: {writer.error or 'encoder shutdown timed out'}", flush=True)
        return 1
    if not writer.written:
        print("No complete FREC frame recorded. Check new firmware, SW4, IP and Gigabit link.", flush=True)
        return 2
    print(f"SAVED {output}: {writer.written} frames, {writer.repeated} repeated; metadata={writer.sidecar}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
