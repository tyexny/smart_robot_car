"""Independent four-color contour detector and measured-only ball tracking.

OpenCV hue is 0..179. Overall roundness uses the convex outline so the small
spikes on massage balls do not destroy circularity. Original-mask solidity,
raw circularity and hole fraction still reject sparse patches and rings.
Class IDs 80..83 extend (never replace) the original COCO/VLM wire slots.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import re
import secrets
import time

from protocol import DetectionBox, OBSERVED

BALL_LABELS = {80: 'Red Ball', 81: 'Yellow Ball', 82: 'Green Ball', 83: 'Blue Ball'}
BALL_BGR = {80: (0, 0, 255), 81: (0, 255, 255), 82: (0, 255, 0), 83: (255, 0, 0)}
DEFAULT_CONFIG = {
    'hsv_ranges': {
        '80': [[[0, 95, 55], [10, 255, 255]], [[170, 95, 55], [179, 255, 255]]],
        '81': [[[18, 90, 60], [35, 255, 255]]],
        '82': [[[36, 75, 45], [85, 255, 255]]],
        '83': [[[90, 85, 45], [130, 255, 255]]],
    },
    'min_area': 140, 'max_area_fraction': .35, 'min_radius': 7,
    'min_circularity': .82, 'min_aspect': .80, 'max_aspect': 1.25,
    'min_circle_fill': .78, 'min_solidity': .70, 'max_hole_fraction': .12,
    'min_raw_circularity': .20,
    'red_core_hue': 3,
    'yellow_core_hue': 20,
    'max_massage_solidity': .970,
    'min_massage_perimeter_ratio': 1.08,
    'min_massage_color_fraction': .75,
    'retention_seconds': 1.2, 'max_gap_seconds': .6, 'confirm_frames': 3,
}


def ball_class_for_goal(description):
    """Accept only an unambiguous ball noun/color, not a red bottle or box."""
    value = description.strip().lower()
    chinese = re.fullmatch(r'(?:一个|一只|一颗|那[个只颗])?([红黄绿蓝])(?:色)?(?:的)?(?:按摩小球|按摩球|带刺球|刺球|凸点球|小球|球|乒乓球|皮球)', value)
    if chinese:
        return 80 + '红黄绿蓝'.index(chinese[1])
    english = re.fullmatch(r'(?:a |the )?(red|yellow|green|blue)(?: small| massage| spiky)? ball', value)
    return 80 + ('red', 'yellow', 'green', 'blue').index(english[1]) if english else None


def load_config(path=None):
    config = json.loads(json.dumps(DEFAULT_CONFIG))
    if path:
        supplied = json.loads(Path(path).read_text(encoding='utf-8'))
        if not isinstance(supplied, dict) or set(supplied) - set(config):
            raise ValueError('Unknown HSV ball configuration keys')
        config.update(supplied)
    positive = ('min_area', 'min_radius', 'retention_seconds', 'max_gap_seconds', 'min_massage_perimeter_ratio')
    fractions = ('max_area_fraction', 'min_circularity', 'min_circle_fill', 'min_solidity',
                 'max_hole_fraction', 'min_raw_circularity', 'max_massage_solidity',
                 'min_massage_color_fraction')
    for key in (*positive, *fractions, 'min_aspect', 'max_aspect'):
        value = config[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid HSV ball setting: {key}')
        if key in fractions and value > 1:
            raise ValueError(f'HSV ball fraction must be <= 1: {key}')
    if config['min_aspect'] > config['max_aspect'] or type(config['confirm_frames']) is not int or not 1 <= config['confirm_frames'] <= 20:
        raise ValueError('Invalid HSV aspect/confirmation settings')
    if type(config['red_core_hue']) is not int or not 0 <= config['red_core_hue'] <= 15:
        raise ValueError('red_core_hue must be an integer in 0..15')
    if type(config['yellow_core_hue']) is not int or not 15 <= config['yellow_core_hue'] <= 35:
        raise ValueError('yellow_core_hue must be an integer in 15..35')
    if not isinstance(config['hsv_ranges'], dict) or set(config['hsv_ranges']) != {'80', '81', '82', '83'}:
        raise ValueError('HSV ranges must specify the four ball classes')
    for ranges in config['hsv_ranges'].values():
        if not isinstance(ranges, list) or not 1 <= len(ranges) <= 4:
            raise ValueError('Invalid HSV range list')
        for bounds in ranges:
            if not isinstance(bounds, list) or len(bounds) != 2 or any(not isinstance(row, list) or len(row) != 3 for row in bounds):
                raise ValueError('HSV bounds must be [[H,S,V],[H,S,V]]')
            for i, high in enumerate((179, 255, 255)):
                low, upper = bounds[0][i], bounds[1][i]
                if type(low) is not int or type(upper) is not int or not 0 <= low <= upper <= high:
                    raise ValueError('Invalid HSV channel bounds')
    return config


@dataclass
class BallTrack:
    box: DetectionBox
    seen_at: float
    hits: int = 1
    vx: float = 0.
    vy: float = 0.


class BallDetector:
    def __init__(self, config_path=None):
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        self.config = load_config(config_path)
        self.ranges = {int(cid): [(np.array(lo, np.uint8), np.array(hi, np.uint8)) for lo, hi in ranges]
                       for cid, ranges in self.config['hsv_ranges'].items()}
        self.reset()

    def reset(self):
        self.session = secrets.randbits(32) or 1
        self.tracks = {}
        # Keep local IDs disjoint from ordinary remote detector IDs. Remote IDs
        # are remapped into the lower half before merging by HybridTaskController.
        self.next_id = 1 << 23
        self.last_capture = None
        self.epoch = None

    def _color_masks(self, hsv, cid):
        cv2, np, cfg = self.cv2, self.np, self.config
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for low, high in self.ranges[cid]:
            mask |= cv2.inRange(hsv, low, high)
        masks = [mask]
        if cid == 80:
            hue = hsv[:, :, 0]
            for upper in (cfg['red_core_hue'], min(15, cfg['red_core_hue']+1)):
                core = ((hue <= upper) | (hue >= 170)).astype(np.uint8)*255
                masks.append(mask & core)
        elif cid == 81:
            core = (hsv[:, :, 0] >= cfg['yellow_core_hue']).astype(np.uint8)*255
            masks.append(mask & core)
        return masks

    def contours(self, image, *, detail_image=None, massage_only=False):
        cv2, np, cfg = self.cv2, self.np, self.config
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        height, width = image.shape[:2]
        detected = []
        for cid in self.ranges:
            # Broad/core alternatives separate red/yellow balls from wooden
            # floors without discarding the original broad color support.
            for candidate_mask in self._color_masks(hsv, cid):
                candidate_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                detected.extend(self._mask_candidates(candidate_mask, cid, width, height))
        # Broad/core masks can describe the same ball. Never publish duplicate
        # identities for those alternative segmentations.
        unique = []
        for box in sorted(detected, key=lambda b:b.confidence, reverse=True):
            if any(old.class_id == box.class_id and _box_iou(old, box) >= .45 for old in unique):
                continue
            unique.append(box)
            if len(unique) == 32:
                break
        if massage_only:
            detail = image if detail_image is None else detail_image
            unique = [box for box in unique if self._is_massage_ball(detail, box, width, height)]
        return unique

    def _is_massage_ball(self, image, box, work_width, work_height):
        """Measured outline bumps; refuse smooth or unresolved ball outlines.

        Read detail from the native frame, not from an enlarged recognition
        view. Choose the most filled valid color component so basketball seams
        cannot create artificial evidence of spikes in a sparse color mask.
        This is a geometric heuristic, not a trained object-type classifier.
        """
        cv2, np, cfg = self.cv2, self.np, self.config
        sy, sx = image.shape[0]/work_height, image.shape[1]/work_width
        x1, y1 = max(0, int((box.x1-2)*sx)), max(0, int((box.y1-2)*sy))
        x2, y2 = min(image.shape[1], int((box.x2+2)*sx)), min(image.shape[0], int((box.y2+2)*sy))
        roi = image[y1:y2, x1:x2]
        if min(roi.shape[:2], default=0) < 24:
            return False
        if max(roi.shape[:2]) > 512:
            scale = 512/max(roi.shape[:2])
            roi = cv2.resize(roi, (round(roi.shape[1]*scale), round(roi.shape[0]*scale)), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        evidence = []
        for mask in self._color_masks(hsv, box.class_id):
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < roi.shape[0]*roi.shape[1]*.30:
                    continue
                hull = cv2.convexHull(contour)
                hull_area, hull_perimeter = cv2.contourArea(hull), cv2.arcLength(hull, True)
                if 4*math.pi*hull_area/max(hull_perimeter**2, 1) < .84:
                    continue
                hull_mask = np.zeros(mask.shape, np.uint8)
                cv2.drawContours(hull_mask, [hull], -1, 255, -1)
                color_fraction = cv2.countNonZero(mask & hull_mask)/max(cv2.countNonZero(hull_mask), 1)
                evidence.append((area/max(hull_area, 1), cv2.arcLength(contour, True)/max(hull_perimeter, 1), color_fraction))
        if not evidence:
            return False
        solidity, perimeter_ratio, color_fraction = max(evidence)
        return (solidity <= cfg['max_massage_solidity'] and
                perimeter_ratio >= cfg['min_massage_perimeter_ratio'] and
                color_fraction >= cfg['min_massage_color_fraction'])

    def _mask_candidates(self, mask, cid, width, height):
        cv2, cfg = self.cv2, self.config
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        found = []
        for i, contour in enumerate(contours):
            if hierarchy[0][i][3] != -1:
                continue
            area = cv2.contourArea(contour)
            if not cfg['min_area'] <= area <= width * height * cfg['max_area_fraction']:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if x <= 0 or y <= 0 or x+w >= width or y+h >= height or not cfg['min_aspect'] <= w/h <= cfg['max_aspect']:
                continue
            perimeter = cv2.arcLength(contour, True)
            raw_circularity = 4 * math.pi * area / max(perimeter**2, 1)
            if raw_circularity < cfg['min_raw_circularity']:
                continue
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            hull_perimeter = cv2.arcLength(hull, True)
            circularity = 4 * math.pi * hull_area / max(hull_perimeter**2, 1)
            if circularity < cfg['min_circularity']:
                continue
            (_, _), radius = cv2.minEnclosingCircle(contour)
            fill = hull_area / max(math.pi * radius**2, 1)
            solidity = area / max(hull_area, 1)
            vertices = len(cv2.approxPolyDP(hull, .02 * hull_perimeter, True))
            if radius < cfg['min_radius'] or fill < cfg['min_circle_fill'] or solidity < cfg['min_solidity'] or vertices < 7:
                continue
            holes, child = 0., hierarchy[0][i][2]
            while child != -1:
                holes += cv2.contourArea(contours[child])
                child = hierarchy[0][child][0]
            if holes / area > cfg['max_hole_fraction']:
                continue
            score = round(min(.99, (circularity + fill + solidity) / 3) * 1000)
            found.append(DetectionBox(x, y, x+w, y+h, score, cid, 0, OBSERVED))
        return found

    def detect(self, image, *, captured_at=None, epoch=0, detail_image=None, massage_only=False):
        now = time.monotonic() if captured_at is None else captured_at
        if self.last_capture is not None:
            if epoch != self.epoch or now - self.last_capture > self.config['max_gap_seconds']:
                self.reset()
            elif now <= self.last_capture:
                return []  # Replayed frames cannot count as confirmations.
        self.epoch, self.last_capture = epoch, now
        retention = self.config['retention_seconds']
        self.tracks = {tid:t for tid,t in self.tracks.items() if now-t.seen_at <= retention}
        detections = self.contours(image, detail_image=detail_image, massage_only=massage_only)
        edges = []
        for index, box in enumerate(detections):
            cx, cy = (box.x1+box.x2)/2, (box.y1+box.y2)/2
            for tid, track in self.tracks.items():
                old, dt = track.box, now-track.seen_at
                if old.class_id != box.class_id:
                    continue
                size = max(old.x2-old.x1, old.y2-old.y1)
                ratio = (box.x2-box.x1) / (old.x2-old.x1)
                px = (old.x1+old.x2)/2 + track.vx * min(dt, .35)
                py = (old.y1+old.y2)/2 + track.vy * min(dt, .35)
                distance = math.hypot(cx-px, cy-py)
                gate = max(18., size*.85) + min(dt, .4)*90
                if .6 <= ratio <= 1.65 and distance <= gate:
                    edges.append((distance/gate + abs(math.log(ratio))*.25, index, tid))
        matched, used = {}, set()
        for cost, index, tid in sorted(edges):
            if index in matched or tid in used:
                continue
            # When two same-color identities are indistinguishable, withdraw
            # their observations instead of silently handing off a locked ID.
            rivals = [c for c,i,t in edges if (i == index and t != tid) or (t == tid and i != index)]
            if rivals and min(rivals) <= cost + .12:
                continue
            matched[index] = tid
            used.add(tid)
        output = []
        for index, box in enumerate(detections):
            tid = matched.get(index)
            if tid is None:
                # Do not invent a replacement ID in an ambiguous existing gate.
                if any(i == index for _,i,_ in edges):
                    continue
                if self.next_id >= (1 << 24):
                    self.reset()
                    return []
                tid, self.next_id = self.next_id, self.next_id + 1
                self.tracks[tid] = BallTrack(replace(box, track_id=tid), now)
            else:
                track = self.tracks[tid]
                old, dt = track.box, max(.001, now-track.seen_at)
                track.vx = .5*track.vx + .5*((box.x1+box.x2-old.x1-old.x2)/2/dt)
                track.vy = .5*track.vy + .5*((box.y1+box.y2-old.y1-old.y2)/2/dt)
                track.hits += 1
                track.box, track.seen_at = replace(box, track_id=tid), now
            track = self.tracks[tid]
            if track.hits >= self.config['confirm_frames']:
                output.append(track.box)
        return output  # Never return predicted-only or absent contours.


def merge_display_boxes(yolo_boxes, ball_boxes):
    """Share the existing eight-box budget while keeping every color visible."""
    balls, colors = [], set()
    for box in sorted(ball_boxes, key=lambda b:b.confidence, reverse=True):
        if box.class_id not in colors:
            balls.append(box)
            colors.add(box.class_id)
    balls = balls[:4]
    return balls + list(yolo_boxes)[:8-len(balls)]


def _box_iou(a, b):
    intersection = max(0, min(a.x2, b.x2)-max(a.x1, b.x1)) * max(0, min(a.y2, b.y2)-max(a.y1, b.y1))
    union = (a.x2-a.x1)*(a.y2-a.y1) + (b.x2-b.x1)*(b.y2-b.y1) - intersection
    return intersection/union if union else 0.
