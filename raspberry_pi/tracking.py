"""Small, dependency-free policies around the upstream BoT-SORT/ByteTrack.

The tracker estimates motion; only measurements from the current input frame
may be returned to the car. This module manages clock-time expiry, ID identity,
and the eight-box transport budget without depending on NumPy or YOLO.
"""

from __future__ import annotations

import math
import secrets
import threading
import time
from dataclasses import dataclass, replace

from protocol import (LOCK_VALID, MAX_BOXES, OBSERVED, TRACKED_VERSION, SEGMENTED_VERSION, VIDEO_HEIGHT,
                      VIDEO_WIDTH, DetectionBox, VideoLine, is_newer)

# Upstream 8.4.159 formats tracking rows as float32. Reset before integer IDs
# cease to be exactly representable, well before the wire protocol's u32 limit.
MAX_EXACT_TRACK_ID = (1 << 24) - 1

# The FPGA holds the locked ID for TARGET_RETAIN_MS (2000 ms in production).
# Within that window a replacement tracker ID may be relabelled back to the old
# one, but only after the old box has really disappeared and a single geometric
# successor has been stable for two frames.
REACQUIRE_DELAY_FRAMES = 2
REACQUIRE_CONFIRM_FRAMES = 2
REACQUIRE_SCORE_THRESHOLD = 0.55
REACQUIRE_SCORE_MARGIN = 0.15
REACQUIRE_SIZE_RATIO = 2.0
REACQUIRE_MAX_RADIUS_FRACTION = 0.42
REACQUIRE_RADIUS_GROWTH_PX_PER_S = 150.0
REACQUIRE_PREDICTION_HORIZON_S = 0.8


@dataclass(frozen=True)
class LockRequest:
    tracker_session: int
    track_id: int


@dataclass
class _LockMemory:
    """Last measured geometry of the locked box, used to gate a new ID."""

    class_id: int
    cx: float
    cy: float
    width: float
    height: float
    seen_at: float
    vx: float = 0.0
    vy: float = 0.0


class LockMailbox:
    """Receive-thread writes and inference-thread reads an immutable lock.

    Controls are consumed from each valid line, even if its frame never becomes
    complete. Reordered older frame/row packets cannot reinstate an older lock.
    The assembler's epoch permits a restarted FPGA sequence to begin at zero.
    """

    def __init__(self, timeout=1.0):
        if timeout <= 0:
            raise ValueError("lock timeout must be positive")
        self.timeout = timeout
        self._mutex = threading.Lock()
        self._epoch = None
        self._frame = None
        self._row = -1
        self._received_at = None
        self._request = None

    def update(self, line: VideoLine, epoch=0, now=None):
        now = time.monotonic() if now is None else now
        with self._mutex:
            if self._epoch != epoch:
                self._epoch, self._frame, self._row = epoch, None, -1
                self._request = None
            if self._frame is not None:
                if line.frame_id == self._frame:
                    if line.line_index <= self._row:
                        return
                elif not is_newer(line.frame_id, self._frame):
                    return
            self._frame, self._row = line.frame_id, line.line_index
            self._received_at = now
            if (line.version in (TRACKED_VERSION, SEGMENTED_VERSION) and line.lock_flags & LOCK_VALID
                    and line.lock_session and line.lock_id):
                self._request = LockRequest(line.lock_session, line.lock_id)
            else:
                self._request = None

    def snapshot(self, now=None) -> LockRequest | None:
        now = time.monotonic() if now is None else now
        with self._mutex:
            if self._received_at is None or now - self._received_at >= self.timeout:
                return None
            return self._request


class TrackingState:
    """Identity and expiry policies expressed in elapsed seconds, not frame count.

    The retention default matches the FPGA's production wait (2000 ms): an ID that
    the tracker still needs must outlive the window during which the car holds it.
    """

    def __init__(self, retention_seconds=2.0, max_gap_seconds=0.6,
                 max_id_history=4096, session_factory=None):
        if retention_seconds <= 0 or max_gap_seconds <= 0 or max_id_history < MAX_BOXES:
            raise ValueError("invalid tracking lifetime configuration")
        self.retention_seconds = retention_seconds
        self.max_gap_seconds = max_gap_seconds
        self.max_id_history = max_id_history
        self._session_factory = session_factory or (lambda: secrets.randbits(32))
        self.session = 0
        self._last_epoch = None
        self._last_sample_at = None
        self._identities: dict[int, tuple[int, float]] = {}
        self._retired: set[int] = set()
        self._lock: _LockMemory | None = None
        self._lock_id = 0
        self._missed_frames = 0
        self._candidate_id = 0
        self._candidate_hits = 0
        self._alias: dict[int, int] = {}
        self.reacquire_count = 0
        self.reset_count = 0
        self.reset_reason = "startup"
        self.new_session("startup")

    def new_session(self, reason):
        old = self.session
        # A broken/injected RNG must fail rather than reuse an identity namespace.
        for _ in range(128):
            candidate = int(self._session_factory()) & 0xFFFFFFFF
            if candidate and candidate != old:
                self.session = candidate
                break
        else:
            raise RuntimeError("unable to allocate a fresh nonzero tracker session")
        self._identities.clear()
        self._retired.clear()
        self._lock = None
        self._lock_id = 0
        self._missed_frames = 0
        self._clear_reacquire()
        self.reset_count += 1
        self.reset_reason = reason

    def prepare_frame(self, frame, now=None) -> bool:
        """Return True when the caller must also reset the upstream tracker.

        Use image sampling time, rather than inference completion time. Skipped
        camera frames do not advance a fake 30-FPS clock. A gap above the explicit
        bound breaks the identity namespace instead of applying one fixed-dt
        Kalman step to an arbitrarily stale image.
        """
        sampled_at = frame.completed_at if now is None else now
        reason = None
        if self._last_epoch is not None and frame.sequence_epoch != self._last_epoch:
            reason = "video-sequence-reset"
        elif self._last_sample_at is not None:
            gap = sampled_at - self._last_sample_at
            if gap <= 0 or gap > self.max_gap_seconds:
                reason = "sample-time-gap"
        if reason:
            self.new_session(reason)
        self._last_epoch = frame.sequence_epoch
        self._last_sample_at = sampled_at
        return reason is not None

    def expired_ids(self, now):
        """Remove expired IDs before association, so stale tracks cannot revive."""
        expired = {track_id for track_id, (_, seen) in self._identities.items()
                   if now - seen > self.retention_seconds}
        for track_id in expired:
            del self._identities[track_id]
        self._retired.update(expired)
        return expired

    def accept_observations(self, boxes, now):
        """Return actual current observations and IDs to evict from tracker pools.

        Once an ID has a class, it cannot silently become a different class.
        Tombstones also prevent expiry or a tracker bug from reviving an ID.
        """
        accepted, retired = [], set()
        seen_this_frame = set()
        for box in boxes:
            if not box.flags & OBSERVED:
                continue
            if not 0 < box.track_id <= MAX_EXACT_TRACK_ID:
                self.new_session("track-id-precision-limit")
                return [], retired, True
            if box.track_id in seen_this_frame:
                self.new_session("duplicate-track-id")
                return [], retired, True
            seen_this_frame.add(box.track_id)
            identity = self._identities.get(box.track_id)
            if box.track_id in self._retired or (identity and identity[0] != box.class_id):
                self._identities.pop(box.track_id, None)
                self._retired.add(box.track_id)
                retired.add(box.track_id)
                continue
            self._identities[box.track_id] = (box.class_id, now)
            accepted.append(box)
        if len(self._identities) + len(self._retired) > self.max_id_history:
            self.new_session("identity-history-limit")
            return [], retired, True
        return accepted, retired, False

    def _clear_reacquire(self):
        self._alias.clear()
        self._candidate_id = 0
        self._candidate_hits = 0

    @property
    def active_alias(self):
        """Return (replacement ID, locked ID) while a relabel is in effect."""
        return next(iter(self._alias.items()), None)

    def forget_lock(self):
        """Drop the remembered lock; the FPGA no longer reports one."""
        self._lock = None
        self._lock_id = 0
        self._missed_frames = 0
        self._clear_reacquire()

    def _remember(self, box, now):
        cx, cy = (box.x1 + box.x2) / 2.0, (box.y1 + box.y2) / 2.0
        width, height = float(box.x2 - box.x1), float(box.y2 - box.y1)
        memory = self._lock
        if memory is None:
            memory = self._lock = _LockMemory(box.class_id, cx, cy, width, height, now)
        elif now > memory.seen_at:
            elapsed = now - memory.seen_at
            vx = (cx - memory.cx) / elapsed
            vy = (cy - memory.cy) / elapsed
            # Smooth one noisy measurement; the search gate also grows with time.
            memory.vx = 0.5 * memory.vx + 0.5 * max(-1500.0, min(1500.0, vx))
            memory.vy = 0.5 * memory.vy + 0.5 * max(-1500.0, min(1500.0, vy))
        memory.class_id = box.class_id
        memory.cx, memory.cy = cx, cy
        memory.width, memory.height = width, height
        memory.seen_at = now

    def _candidate_score(self, box, now):
        """Return 0..1 for one box, or None when it cannot be the same target."""
        memory = self._lock
        horizon = min(max(0.0, now - memory.seen_at), REACQUIRE_PREDICTION_HORIZON_S)
        predicted_x = min(max(memory.cx + memory.vx * horizon, 0.0), VIDEO_WIDTH - 1.0)
        predicted_y = min(max(memory.cy + memory.vy * horizon, 0.0), VIDEO_HEIGHT - 1.0)
        cx, cy = (box.x1 + box.x2) / 2.0, (box.y1 + box.y2) / 2.0
        radius = min(96.0 + 0.5 * memory.width + REACQUIRE_RADIUS_GROWTH_PX_PER_S * horizon,
                     REACQUIRE_MAX_RADIUS_FRACTION * VIDEO_WIDTH)
        distance = math.hypot(cx - predicted_x, cy - predicted_y)
        if distance > radius:
            return None
        width, height = float(box.x2 - box.x1), float(box.y2 - box.y1)
        if width <= 0 or height <= 0:
            return None
        ratio = max(max(width / memory.width, memory.width / width),
                    max(height / memory.height, memory.height / height))
        if ratio > REACQUIRE_SIZE_RATIO:
            return None
        return 0.6 * (1.0 - distance / radius) + 0.4 / ratio

    def reacquire(self, boxes, lock, now, *, strong_confidence=350):
        """Relabel a replacement tracker ID back to the ID the FPGA still holds.

        The car keeps waiting for one ID for TARGET_RETAIN_MS; a tracker that
        renumbers its target would otherwise force the operator to reselect it.
        Only a single, stable, same-class geometric successor may take the old
        number, and the locked identity is refreshed while it does, so it cannot
        expire in the middle of the search.
        """
        boxes = list(boxes)
        if lock is None or lock.tracker_session != self.session:
            self.forget_lock()
            return boxes
        if lock.track_id != self._lock_id:
            self.forget_lock()
            self._lock_id = lock.track_id
        if lock.track_id in self._retired:
            self.forget_lock()
            return boxes
        locked = next((box for box in boxes if box.track_id == lock.track_id), None)
        if locked is not None:
            self._clear_reacquire()
            self._missed_frames = 0
            self._remember(locked, now)
            return boxes
        if self._lock is None:
            return boxes
        if self._alias:
            current = next((box for box in boxes if box.track_id in self._alias), None)
            if current is not None:
                score = (self._candidate_score(current, now)
                         if current.flags & OBSERVED and current.class_id == self._lock.class_id
                         else None)
                if score is not None and score >= REACQUIRE_SCORE_THRESHOLD:
                    # Keep a valid incumbent instead of competing for a new ID.
                    self._missed_frames = 0
                    self._candidate_id = self._candidate_hits = 0
                    if current.confidence >= strong_confidence:
                        self._remember(current, now)
                    # Weak real observations keep identity, not motion authority.
                    return self._apply_alias(boxes, now)
                # Do not let fallback paths relabel a bad incumbent observation.
                self._clear_reacquire()
        self._missed_frames += 1
        if self._missed_frames < REACQUIRE_DELAY_FRAMES:
            return self._apply_alias(boxes, now)
        scored = []
        for box in boxes:
            if (not box.flags & OBSERVED or not box.track_id or box.track_id == lock.track_id
                    or box.track_id in self._alias):
                continue
            if box.class_id != self._lock.class_id or box.confidence < strong_confidence:
                continue
            score = self._candidate_score(box, now)
            if score is not None:
                scored.append((score, box))
        scored.sort(key=lambda item: (-item[0], item[1].track_id))
        if len(scored) > 1 and scored[0][0] - scored[1][0] < REACQUIRE_SCORE_MARGIN:
            scored = []  # two equally plausible boxes: guessing here could hijack the car
        if not scored or scored[0][0] < REACQUIRE_SCORE_THRESHOLD:
            self._candidate_id = 0
            self._candidate_hits = 0
            return self._apply_alias(boxes, now)
        candidate = scored[0][1]
        if candidate.track_id == self._candidate_id:
            self._candidate_hits += 1
        else:
            self._candidate_id, self._candidate_hits = candidate.track_id, 1
        if self._candidate_hits < REACQUIRE_CONFIRM_FRAMES:
            return self._apply_alias(boxes, now)
        self._alias = {candidate.track_id: lock.track_id}
        self._candidate_id = 0
        self._candidate_hits = 0
        self.reacquire_count += 1
        self._missed_frames = 0
        self._remember(candidate, now)
        return self._apply_alias(boxes, now)

    def _apply_alias(self, boxes, now):
        if not self._alias:
            return boxes
        rewritten, claimed = [], set()
        for box in boxes:
            old = self._alias.get(box.track_id)
            if old is None:
                rewritten.append(box)
                continue
            identity = self._identities.get(old)
            if old in self._retired or (identity is not None and identity[0] != box.class_id):
                # A completed or reclassified identity must not be revived here.
                self._clear_reacquire()
                return boxes
            if old in claimed:
                continue
            claimed.add(old)
            self._identities[old] = (box.class_id, now)
            rewritten.append(replace(box, track_id=old))
        return rewritten


def evict_track_ids(trackers, retired_ids):
    """Forget dead identities in the pinned upstream tracked/lost pools.

    IDs are monotonically allocated by the upstream tracker within a session.
    Keep its removed pool bounded; our own tombstones make reuse impossible.
    """
    if not retired_ids:
        return
    for tracker in trackers:
        removed = []
        for name in ("tracked_stracks", "lost_stracks"):
            pool = getattr(tracker, name, [])
            retained = []
            for track in pool:
                if int(track.track_id) in retired_ids:
                    track.mark_removed()
                    removed.append(track)
                else:
                    retained.append(track)
            setattr(tracker, name, retained)
        tracker.removed_stracks = (getattr(tracker, "removed_stracks", []) + removed)[-1000:]


def choose_reply_boxes(boxes, tracker_session, lock=None, *, strong_confidence=350,
                       preferred_class=None):
    """Reserve the first slot for the current lock, then fill by confidence.

    Low-score observations keep an established identity alive, but remain below
    the FPGA's drive threshold. Predictions and stale-session locks are ignored.
    Without a current lock, every strong candidate remains eligible for selection.
    """
    observed = [box for box in boxes if box.flags & OBSERVED and box.track_id]
    ordered = sorted(observed, key=lambda box: (
        box.class_id != preferred_class if preferred_class is not None else False,
        -box.confidence, box.track_id))
    locked = None
    if lock and lock.tracker_session == tracker_session:
        locked = next((box for box in ordered if box.track_id == lock.track_id), None)
    selected = [locked] if locked else []
    for box in ordered:
        if box is locked or box.confidence < strong_confidence:
            continue
        selected.append(box)
        if len(selected) == MAX_BOXES:
            break
    return selected
