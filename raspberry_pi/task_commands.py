"""Small, deterministic Chinese task interpreter for the FPGA motion link."""

from __future__ import annotations

import logging
import secrets
import struct
import threading
import time

from protocol import OBSERVED

LOG = logging.getLogger("fpga_yolo.task")
WIRE = struct.Struct("!4sBBII")
MOTION = {
    "前进": 0x40, "后退": 0x19, "停车": 0x15,
    "左转弯": 0x44, "右转弯": 0x43,
    "原地左转": 0x16, "原地右转": 0x0D,
    "左平移": 0x07, "右平移": 0x09,
    "加速": 0x45, "减速": 0x47,
}
DIRECTION_CODES = frozenset(MOTION[name] for name in (
    "前进", "后退", "左转弯", "右转弯", "原地左转", "原地右转", "左平移", "右平移"))
MANUAL_REFRESH_SECONDS = 0.25
MANUAL_KEEPALIVE = 0xF0
TARGETS = {
    "跟踪识别到的人": (0, False), "跟踪人": (0, False),
    "跟踪识别到的瓶子": (39, False), "跟踪瓶子": (39, False),
    "移动到人附近": (0, True), "移动到瓶子附近": (39, True),
    "找到人并移动到其附近": (0, True),
    "找到瓶子并移动到其附近": (39, True),
}


def parse_text(text: str):
    """Exact phrases only; avoid accidentally moving on partial ASR text."""
    text = "".join(text.strip().split()).rstrip("。！!，,")
    if text in MOTION:
        return ("motion", MOTION[text])
    if text in TARGETS:
        return ("target", *TARGETS[text])
    return None


class TaskController:
    def __init__(self, udp_socket, destination, clock=time.monotonic):
        self.socket = udp_socket
        self.destination = destination
        self.clock = clock
        self.session = secrets.randbits(32) or 1
        self.sequence = 0
        self.target_class = None
        self.approach = False
        self.phase = "idle"
        self.started = 0.0
        self.last_search_turn = 0.0
        self.selected_track = 0
        self.selection_frame = None
        self.manual_opcode = None
        self.manual_refreshed = 0.0
        self._send_lock = threading.RLock()

    def send(self, opcode):
        with self._send_lock:
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            self.socket.sendto(WIRE.pack(b"TCMD", 1, opcode, self.session, self.sequence),
                               self.destination)

    def refresh_manual_motion(self):
        """Keep an explicit manual direction active while the Pi process runs."""
        with self._send_lock:
            now = self.clock()
            if self.manual_opcode is not None and now - self.manual_refreshed >= MANUAL_REFRESH_SECONDS:
                self.send(MANUAL_KEEPALIVE)
                self.manual_refreshed = now

    def submit(self, text):
        intent = parse_text(text)
        if intent is None:
            LOG.warning("Unknown command: %s", text)
            return False
        with self._send_lock:
            if intent[0] == "motion":
                self.phase = "idle"
                self.target_class = None
                if intent[1] in DIRECTION_CODES:
                    self.manual_opcode = intent[1]
                    self.manual_refreshed = self.clock()
                elif intent[1] == MOTION["停车"]:
                    self.manual_opcode = None
                self.send(intent[1])
                LOG.info("Command: %s", text)
            else:
                self.manual_opcode = None
                self.target_class, self.approach = intent[1:]
                self.phase = "search"
                self.started = self.clock()
                self.last_search_turn = 0.0
                self.selected_track = 0
                self.selection_frame = None
                self.send(MOTION["停车"])
                LOG.info("Searching for %s", "person" if self.target_class == 0 else "bottle")
        return True

    def prepare_boxes(self, boxes, frame_id):
        """Return the only candidate during selection; later return all detections."""
        if self.phase == "idle" or self.phase == "following":
            return boxes
        now = self.clock()
        if now - self.started > 15.0:
            self.send(MOTION["停车"])
            self.phase = "idle"
            LOG.warning("Target search timed out")
            return boxes
        candidates = [box for box in boxes if box.class_id == self.target_class
                      and box.track_id and box.flags & OBSERVED and box.confidence >= 350]
        if self.phase == "search":
            if candidates:
                chosen = max(candidates, key=lambda box: box.confidence)
                self.selected_track = chosen.track_id
                self.phase = "select"
                self.selection_frame = frame_id
                self.send(0x46)
                LOG.info("Selected %s ID %d confidence %.3f",
                         "person" if self.target_class == 0 else "bottle",
                         chosen.track_id, chosen.confidence / 1000)
                return [chosen]
            if now - self.last_search_turn >= 0.7:
                self.send(MOTION["原地左转"])
                self.last_search_turn = now
            return boxes
        selected = next((box for box in candidates if box.track_id == self.selected_track), None)
        if selected is None:
            self.phase = "search"
            self.selected_track = 0
            self.send(MOTION["停车"])
            return boxes
        return [selected]

    def after_boxes_sent(self, frame_id):
        if self.phase == "select" and frame_id != self.selection_frame:
            self.send(0xA1 if self.approach else 0x1C)
            self.phase = "following"
            LOG.info("%s ID %d", "Approaching" if self.approach else "Following",
                     self.selected_track)
