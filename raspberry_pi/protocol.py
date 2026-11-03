"""Versioned FPGA/Pi wire protocol, in network byte order.

V1 packets stay byte-for-byte compatible. V2 adds a tracker session/ID and
per-line lock feedback; prediction-only boxes never need to cross the wire.
"""

from __future__ import annotations

import math
import struct
import time
from collections import Counter
from dataclasses import dataclass
from numbers import Integral

VIDEO_MAGIC = b"FVID"
BOX_MAGIC = b"FBOX"
VERSION = 1
TRACKED_VERSION = 2
SEGMENTED_VERSION = 3
RGB565_BE = 1
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 360
MAX_BOXES = 8
COCO_CLASS_COUNT = 80
WIRE_CLASS_COUNT = 84  # 80..83: Pi-local Red/Yellow/Green/Blue Ball
VIDEO_HEADER = struct.Struct("!4sBBHIHHHHI")
BOX_HEADER = struct.Struct("!4sBBHIHH")
BOX_RECORD = struct.Struct("!6H")
VIDEO_HEADER_V2 = struct.Struct("!4sBBHIHHHHIII")
VIDEO_HEADER_V3 = struct.Struct("!4sBBHIHHHHIIIHH")
BOX_HEADER_V2 = struct.Struct("!4sBBHIHHI")
BOX_HEADER_V3 = struct.Struct("!4sBBHIHHIB15s")
BOX_RECORD_V2 = struct.Struct("!6HIH")
OBSERVED = 1
LOCK_VALID = 1


class ProtocolError(ValueError):
    """A datagram cannot be used by this protocol."""


def is_newer(candidate: int, reference: int) -> bool:
    """Compare wrapping 32-bit sequence numbers, assuming a gap < 2**31."""
    distance = (candidate - reference) & 0xFFFFFFFF
    return 0 < distance < 0x80000000


@dataclass(frozen=True)
class VideoLine:
    frame_id: int
    width: int
    height: int
    line_index: int
    pixels: bytes
    version: int = VERSION
    lock_session: int = 0
    lock_id: int = 0
    lock_flags: int = 0
    column_index: int = 0


@dataclass(frozen=True)
class VideoFrame:
    frame_id: int
    width: int
    height: int
    pixels: bytes
    first_packet_at: float
    completed_at: float
    version: int = VERSION
    sequence_epoch: int = 0
    source: str = "camera"


@dataclass(frozen=True)
class DetectionBox:
    """Coordinates use the input image; x2/y2 are exclusive right/bottom."""

    x1: int
    y1: int
    x2: int
    y2: int
    confidence: int = 1000
    class_id: int = 0
    track_id: int = 0
    flags: int = 0


@dataclass(frozen=True)
class BoxPacket:
    frame_id: int
    width: int
    height: int
    boxes: list[DetectionBox]
    version: int = VERSION
    tracker_session: int = 0
    label: str = ""


def decode_video_line(packet: bytes, width=VIDEO_WIDTH, height=VIDEO_HEIGHT) -> VideoLine:
    if len(packet) < VIDEO_HEADER.size:
        raise ProtocolError("truncated video header")
    magic, version, pixel_format, header_len = struct.unpack_from("!4sBBH", packet)
    if magic != VIDEO_MAGIC or pixel_format != RGB565_BE:
        raise ProtocolError("unsupported video header")
    lock_session = lock_id = lock_flags = 0
    if version == VERSION and header_len == VIDEO_HEADER.size:
        _, _, _, _, frame_id, w, h, y, count, reserved = VIDEO_HEADER.unpack_from(packet)
        if reserved:
            raise ProtocolError("nonzero video reserved field")
    elif ((version == TRACKED_VERSION and header_len == VIDEO_HEADER_V2.size) or
          (version == SEGMENTED_VERSION and header_len == VIDEO_HEADER_V3.size)):
        if len(packet) < header_len:
            raise ProtocolError("truncated v2 video header")
        (_, _, _, _, frame_id, w, h, y, count, lock_session, lock_id,
         lock_flags) = VIDEO_HEADER_V2.unpack_from(packet)
        if lock_flags & ~LOCK_VALID or (lock_flags & LOCK_VALID and not (lock_session and lock_id)):
            raise ProtocolError("invalid video lock control")
    else:
        raise ProtocolError("unsupported video version/header length")
    if width is None and height is None:
        expected = (1280, 720) if version == SEGMENTED_VERSION else (VIDEO_WIDTH, VIDEO_HEIGHT)
    else:
        expected = (width, height)
    if (w, h) != expected or count != 1 or y >= h or w <= 0 or h <= 0:
        raise ProtocolError("wrong geometry or line index/count")
    column, pixels = 0, w
    if version == SEGMENTED_VERSION:
        column, pixels = struct.unpack_from("!HH", packet, VIDEO_HEADER_V2.size)
        if w % 2 or pixels != w // 2 or column not in (0, w // 2):
            raise ProtocolError("wrong row segment")
    if len(packet) != header_len + pixels * 2:
        raise ProtocolError("wrong video payload length")
    return VideoLine(frame_id, w, h, y, packet[header_len:], version,
                     lock_session, lock_id, lock_flags, column)


def encode_video_line(frame_id: int, line_index: int, pixels: bytes,
                      width=VIDEO_WIDTH, height=VIDEO_HEIGHT, *, version=VERSION,
                      lock_session=0, lock_id=0, lock_flags=0, column_index=0) -> bytes:
    """Used by the simulator/tests; the FPGA emits the same format."""
    segment_pixels = width // 2 if version == SEGMENTED_VERSION else width
    if (len(pixels) != segment_pixels * 2 or not 0 <= line_index < height or
            column_index not in ((0, width // 2) if version == SEGMENTED_VERSION else (0,)) or
            (version == SEGMENTED_VERSION and width % 2)):
        raise ProtocolError("invalid simulator line")
    if version in (TRACKED_VERSION, SEGMENTED_VERSION):
        if not (0 <= lock_session <= 0xFFFFFFFF and 0 <= lock_id <= 0xFFFFFFFF):
            raise ProtocolError("lock session/ID must fit u32")
        if lock_flags & ~LOCK_VALID or (lock_flags & LOCK_VALID and not (lock_session and lock_id)):
            raise ProtocolError("invalid lock control")
        header = VIDEO_HEADER_V2.pack(VIDEO_MAGIC, version, RGB565_BE,
                                    VIDEO_HEADER_V3.size if version == SEGMENTED_VERSION else VIDEO_HEADER_V2.size,
                                    frame_id & 0xFFFFFFFF, width, height, line_index, 1,
                                    lock_session, lock_id, lock_flags)
        if version == SEGMENTED_VERSION:
            header += struct.pack("!HH", column_index, segment_pixels)
        return header + pixels
    if version != VERSION or lock_session or lock_id or lock_flags:
        raise ProtocolError("v1 has no lock control")
    return VIDEO_HEADER.pack(VIDEO_MAGIC, VERSION, RGB565_BE, VIDEO_HEADER.size,
                            frame_id & 0xFFFFFFFF, width, height, line_index, 1, 0) + pixels


def encode_boxes(frame_id: int, boxes: list[DetectionBox],
                 width=VIDEO_WIDTH, height=VIDEO_HEIGHT, *, tracker_session=0, label="") -> bytes:
    if len(boxes) > MAX_BOXES:
        raise ProtocolError("at most eight boxes are supported")
    if not isinstance(tracker_session, Integral) or not 0 <= tracker_session <= 0xFFFFFFFF:
        raise ProtocolError("tracker_session must fit u32")
    tracked = tracker_session != 0
    if label:
        if (not tracked or not isinstance(label,str) or len(label)>15 or
                any(not 32<=ord(c)<=126 for c in label)):
            raise ProtocolError("VLM label requires a tracked packet and 1..15 printable ASCII characters")
        payload=bytearray(BOX_HEADER_V3.pack(BOX_MAGIC,3,len(boxes),BOX_HEADER_V3.size,
            frame_id&0xffffffff,width,height,tracker_session,len(label),label.encode('ascii').ljust(15,b'\0')))
    elif tracked:
        payload = bytearray(BOX_HEADER_V2.pack(BOX_MAGIC, TRACKED_VERSION, len(boxes),
                                              BOX_HEADER_V2.size, frame_id & 0xFFFFFFFF,
                                              width, height, tracker_session))
    else:
        payload = bytearray(BOX_HEADER.pack(BOX_MAGIC, VERSION, len(boxes), BOX_HEADER.size,
                                           frame_id & 0xFFFFFFFF, width, height))
    used_ids = set()
    for box in boxes:
        if not (0 <= box.x1 < box.x2 <= width and 0 <= box.y1 < box.y2 <= height):
            raise ProtocolError("box coordinates outside the video image")
        if not 0 <= box.confidence <= 1000:
            raise ProtocolError("box confidence must be 0..1000")
        if not isinstance(box.class_id, Integral) or not 0 <= box.class_id < WIRE_CLASS_COUNT:
            raise ProtocolError("class_id must be an integer in 0..83")
        if tracked:
            if not isinstance(box.track_id, Integral) or not 0 < box.track_id <= 0xFFFFFFFF:
                raise ProtocolError("v2 track_id must be a nonzero u32")
            if box.track_id in used_ids:
                raise ProtocolError("duplicate v2 track_id")
            used_ids.add(box.track_id)
            if not isinstance(box.flags, Integral) or box.flags & ~OBSERVED or box.flags < 0:
                raise ProtocolError("invalid v2 box flags")
            payload.extend(BOX_RECORD_V2.pack(box.x1, box.y1, box.x2, box.y2,
                                              box.confidence, box.class_id, box.track_id, box.flags))
        else:
            if box.track_id or box.flags:
                raise ProtocolError("v1 cannot carry track IDs/flags")
            payload.extend(BOX_RECORD.pack(box.x1, box.y1, box.x2, box.y2,
                                          box.confidence, box.class_id))
    return bytes(payload)


def decode_box_packet(packet: bytes) -> BoxPacket:
    """Strict decoder for integration checks and the FPGA simulator."""
    if len(packet) < BOX_HEADER.size:
        raise ProtocolError("truncated box header")
    magic, version, count, header_len, frame_id, width, height = BOX_HEADER.unpack_from(packet)
    if magic != BOX_MAGIC:
        raise ProtocolError("unsupported box header")
    session = 0
    label = ""
    if version == VERSION and header_len == BOX_HEADER.size:
        record = BOX_RECORD
    elif version == TRACKED_VERSION and header_len == BOX_HEADER_V2.size:
        if len(packet) < header_len:
            raise ProtocolError("truncated v2 box header")
        session = BOX_HEADER_V2.unpack_from(packet)[-1]
        if not session:
            raise ProtocolError("v2 tracker session must be nonzero")
        record = BOX_RECORD_V2
    elif version==3 and header_len==BOX_HEADER_V3.size:
        if len(packet)<header_len:
            raise ProtocolError("truncated v3 box label")
        session,length,text=BOX_HEADER_V3.unpack_from(packet)[-3:]
        if (not session or not 1<=length<=15 or any(text[length:]) or
                any(not 32<=v<=126 for v in text[:length])):
            raise ProtocolError("invalid box label")
        label=text[:length].decode('ascii')
        record=BOX_RECORD_V2
    else:
        raise ProtocolError("unsupported box version/header length")
    if count > MAX_BOXES or len(packet) != header_len + count * record.size:
        raise ProtocolError("wrong box count/length")
    boxes = [DetectionBox(*record.unpack_from(packet, header_len + i * record.size))
             for i in range(count)]
    # Apply precisely the same validation as the encoder.
    encode_boxes(frame_id, boxes, width, height, tracker_session=session,label=label)
    return BoxPacket(frame_id, width, height, boxes, version, session,label)


def decode_boxes(packet: bytes) -> tuple[int, int, int, list[DetectionBox]]:
    """Keep the v1 simulator API; use decode_box_packet for session metadata."""
    decoded = decode_box_packet(packet)
    return decoded.frame_id, decoded.width, decoded.height, decoded.boxes


def box_from_xyxy(xyxy, confidence: float, width=VIDEO_WIDTH, height=VIDEO_HEIGHT, class_id=0):
    """Clip YOLO floating coordinates to a valid exclusive integer rectangle."""
    values = [float(v) for v in xyxy]
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        return None
    if not math.isfinite(confidence):
        return None
    try:
        numeric_class = float(class_id)
    except (TypeError, ValueError, OverflowError):
        return None
    if not math.isfinite(numeric_class) or not numeric_class.is_integer():
        return None
    class_id = int(numeric_class)
    if not 0 <= class_id < COCO_CLASS_COUNT:
        return None
    x1 = max(0, min(width, math.floor(values[0])))
    y1 = max(0, min(height, math.floor(values[1])))
    x2 = max(0, min(width, math.ceil(values[2])))
    y2 = max(0, min(height, math.ceil(values[3])))
    if x1 >= x2 or y1 >= y2:
        return None
    return DetectionBox(x1, y1, x2, y2,
                        max(0, min(1000, round(confidence * 1000))), class_id)


@dataclass
class _PendingFrame:
    data: bytearray
    seen: bytearray
    first_packet_at: float
    version: int = VERSION
    rows_received: int = 0


class FrameAssembler:
    """Bounded, out-of-order line assembler. Partial frames never leave this object.

    Identical duplicates are ignored. Conflicting duplicates invalidate the whole
    frame. Old complete frames cannot overtake a newer complete frame. An idle
    stream, or a stalled sequence with no newly accepted complete frame, permits
    a restarted FPGA to begin again with frame_id zero.
    """

    def __init__(self, width=VIDEO_WIDTH, height=VIDEO_HEIGHT,
                 frame_timeout=0.25, max_pending=4, sequence_reset_timeout=2.0,
                 auto_geometry=False):
        if width <= 0 or height <= 0 or frame_timeout <= 0 or max_pending < 1:
            raise ValueError("invalid assembler configuration")
        if sequence_reset_timeout <= frame_timeout:
            raise ValueError("sequence reset timeout must exceed frame timeout")
        self.width, self.height = width, height
        self.auto_geometry = auto_geometry
        self._geometry_locked = not auto_geometry
        self._segmented_profile = None
        self.frame_timeout = frame_timeout
        self.max_pending = max_pending
        self.sequence_reset_timeout = sequence_reset_timeout
        self._pending: dict[int, _PendingFrame] = {}
        self._invalid: dict[int, float] = {}
        self._last_completed: int | None = None
        self._last_completed_at: float | None = None
        self._last_valid_packet_at: float | None = None
        self.stats = Counter({name: 0 for name in (
            "packets", "malformed", "duplicate", "conflicting", "old",
            "expired", "evicted", "complete", "sequence_resets")})

    @property
    def geometry(self):
        return (self.width, self.height) if self._geometry_locked else None

    def expire(self, now=None):
        now = time.monotonic() if now is None else now
        for frame_id, pending in list(self._pending.items()):
            if now - pending.first_packet_at >= self.frame_timeout:
                del self._pending[frame_id]
                self._invalid[frame_id] = now + self.frame_timeout
                self.stats["expired"] += 1
        for frame_id, deadline in list(self._invalid.items()):
            if now >= deadline:
                del self._invalid[frame_id]

    def feed(self, packet: bytes, now=None) -> VideoFrame | None:
        now = time.monotonic() if now is None else now
        self.stats["packets"] += 1
        try:
            line = decode_video_line(packet, *(self.geometry or (None, None)))
        except ProtocolError:
            self.stats["malformed"] += 1
            self.expire(now)
            return None
        return self.feed_line(line, now, count_packet=False)

    def feed_line(self, line: VideoLine, now=None, *, count_packet=True) -> VideoFrame | None:
        """Consume an already validated row; receiver can inspect control immediately."""
        now = time.monotonic() if now is None else now
        if count_packet:
            self.stats["packets"] += 1
        segmented = line.version == SEGMENTED_VERSION
        segments = 2 if segmented else 1
        pixels = line.width // segments
        shape_ok = ((line.width, line.height) in ((640, 360), (1280, 720)) and
                    segmented == ((line.width, line.height) == (1280, 720)))
        geometry_ok = ((line.width, line.height) == (self.width, self.height)
                       if self._geometry_locked else shape_ok)
        if (not geometry_ok or line.width % segments or
                not 0 <= line.line_index < line.height or
                line.column_index not in ((0, pixels) if segmented else (0,)) or
                len(line.pixels) != pixels * 2 or
                (self.auto_geometry and self._segmented_profile is not None and
                 self._segmented_profile != segmented)):
            self.stats['malformed'] += 1
            self.expire(now)
            return None
        if not self._geometry_locked:
            self.width, self.height = line.width, line.height
            self._geometry_locked = True
        if self.auto_geometry:
            self._segmented_profile = segmented
        stream_idle = (self._last_valid_packet_at is not None
                       and now - self._last_valid_packet_at >= self.sequence_reset_timeout)
        sequence_stalled = (self._last_completed is not None
                            and not is_newer(line.frame_id, self._last_completed)
                            and self._last_completed_at is not None
                            and now - self._last_completed_at >= self.sequence_reset_timeout)
        if stream_idle or sequence_stalled:
            self._pending.clear()
            self._invalid.clear()
            self._last_completed = None
            self._last_completed_at = None
            self.stats["sequence_resets"] += 1
        self._last_valid_packet_at = now
        self.expire(now)
        if line.frame_id in self._invalid:
            return None
        if self._last_completed is not None and not is_newer(line.frame_id, self._last_completed):
            self.stats["old"] += 1
            return None
        pending = self._pending.get(line.frame_id)
        if pending is None:
            if len(self._pending) >= self.max_pending:
                # Preserve newer frames if packets from an older frame arrive late.
                if all(is_newer(fid, line.frame_id) for fid in self._pending):
                    self.stats["old"] += 1
                    return None
                oldest = min(self._pending, key=lambda fid: self._pending[fid].first_packet_at)
                del self._pending[oldest]
                self._invalid[oldest] = now + self.frame_timeout
                self.stats["evicted"] += 1
            pending = _PendingFrame(bytearray(self.width * self.height * 2),
                                    bytearray(self.height * segments), now, line.version)
            self._pending[line.frame_id] = pending
        if pending.version != line.version:
            del self._pending[line.frame_id]
            self._invalid[line.frame_id] = now + self.frame_timeout
            self.stats["conflicting"] += 1
            return None
        start = (line.line_index * self.width + line.column_index) * 2
        end = start + len(line.pixels)
        piece = line.line_index * segments + (1 if segmented and line.column_index else 0)
        if pending.seen[piece]:
            if pending.data[start:end] == line.pixels:
                self.stats["duplicate"] += 1
            else:
                del self._pending[line.frame_id]
                self._invalid[line.frame_id] = now + self.frame_timeout
                self.stats["conflicting"] += 1
            return None
        pending.data[start:end] = line.pixels
        pending.seen[piece] = 1
        pending.rows_received += 1
        if pending.rows_received != self.height * segments:
            return None
        frame = VideoFrame(line.frame_id, self.width, self.height, bytes(pending.data),
                           pending.first_packet_at, now, pending.version,
                           self.stats["sequence_resets"])
        self._last_completed = line.frame_id
        self._last_completed_at = now
        self.stats["complete"] += 1
        # Discard partial older frames as soon as a newer complete frame is ready.
        for frame_id in list(self._pending):
            if not is_newer(frame_id, line.frame_id):
                del self._pending[frame_id]
        return frame
