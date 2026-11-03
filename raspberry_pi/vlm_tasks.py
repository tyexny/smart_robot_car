"""Open-vocabulary visual tasks over the existing FBOX/TCMD FPGA link.

The VLM only proposes a box.  Motion uses a fresh visual observation, a
bounded template tracker and the FPGA's existing obstacle/distance interlocks.
"""

from __future__ import annotations

import base64
import json
import logging
import queue
import re
import secrets
import threading
import time
import urllib.request
from dataclasses import dataclass

from protocol import DetectionBox, OBSERVED
from task_commands import MOTION

LOG = logging.getLogger("fpga_yolo.vlm")
VLM_WIRE_CLASS = 79  # Valid FBOX class unused by the deployed 13-class YOLO.
RECHECK_INTERVAL = 1.0
RECHECK_TIMEOUT = 12.0
ACQUISITION_RECHECK_INTERVAL = 6.0
SEARCH_TURN_SECONDS = 0.55
SEARCH_SETTLE_SECONDS = 0.35
CHROMATIC_GOALS = {"红色": ((0, 12), (170, 179)), "黄色": ((18, 42),),
                   "绿色": ((43, 85),), "蓝色": ((90, 135),)}


@dataclass(frozen=True)
class VisualGoal:
    description: str
    approach: bool
    thermal_check: bool = False


def english_goal_label(description):
    """Fallback for a missing model label; never substitute an unrelated COCO name."""
    if description.isascii():
        return clean_english_label(description)
    nouns=(("水瓶","bottle"),("瓶子","bottle"),("杯子","cup"),("杯","cup"),
           ("书包","backpack"),("背包","backpack"),("雨伞","umbrella"),
           ("手机","cell phone"),("苹果","apple"),("香蕉","banana"),("椅子","chair"),
           ("书","book"),("球","ball"),("盒子","box"),("箱子","box"),("人","person"))
    nouns=(("遥控器","remote"),("键盘","keyboard"),("鼠标","mouse"),
           ("显示器","monitor"),("屏幕","screen"),("笔记本电脑","laptop"),("电脑","computer"),
           ("剪刀","scissors"),("眼镜","glasses"),("勺子","spoon"),("叉子","fork"),
           ("盘子","plate"),("碗","bowl"),("花盆","plant pot"),("花","flower"),
           ("毛巾","towel"),("鞋","shoe"),("玩具","toy"),("猫","cat"),("狗","dog"),
           ("橙子","orange"),("梨","pear"),("桌子","table"),("纸巾","tissue"),
           ("笔","pen"),("门","door"),("灯","lamp"))+nouns
    noun=next((english for chinese,english in nouns if chinese in description),None)
    if noun is None:
        return "object"
    color=next((english for chinese,english in (("红","red"),("黄","yellow"),
        ("绿","green"),("蓝","blue"),("黑","black"),("白","white")) if chinese in description),"")
    return clean_english_label((color+' '+noun).strip())


def clean_english_label(label, fallback="object"):
    """Wire/display labels use at most 15 printable ASCII characters."""
    if not isinstance(label,str) or not label.isascii():
        return fallback
    label=' '.join(re.sub(r'[^A-Za-z0-9 -]',' ',label).split())
    if (label.lower() in ("vlm object","unknown","unknown object","object") or
            re.search(r'\btargets?\b',label,re.IGNORECASE)):
        return fallback
    return label[:15].rstrip() or fallback


class VlmBoxes(list):
    """Coordinate-list compatibility, plus labels keyed by accepted geometry."""
    def __init__(self):
        super().__init__()
        self.labels={}


@dataclass(frozen=True)
class YoloObservation:
    image: object
    boxes: tuple[DetectionBox, ...]
    frame_id: int
    captured_at: float
    tracker_session: int


def yolo_class_for_goal(description):
    """Only map descriptions whose noun is an existing detector class."""
    for names, class_id in ((("书包", "背包"), 24), (("水瓶", "瓶子"), 39),
                            (("足球", "篮球", "排球"), 32)):
        if any(name in description for name in names):
            return class_id
    if description in ("人", "行人", "男人", "女人", "老人", "小孩") or description.endswith("的人"):
        return 0
    return None


def matching_recheck_box(reference, candidates):
    """Compare boxes in the same captured frame before trusting a VLM recheck."""
    return any(box_iou(reference, candidate) >= 0.25 for candidate in candidates)


def current_recheck_box(reference, current, candidate, width, height):
    """Transfer a VLM box from its capture frame using the tracked displacement."""
    dx = round((current[0] + current[2] - reference[0] - reference[2]) / 2)
    dy = round((current[1] + current[3] - reference[1] - reference[3]) / 2)
    x1, y1, x2, y2 = candidate
    x1, x2 = x1 + dx, x2 + dx
    y1, y2 = y1 + dy, y2 + dy
    # Preserve box size when it meets a frame edge.
    x1, x2 = x1 - max(0, x2 - width), x2 - max(0, x2 - width)
    y1, y2 = y1 - max(0, y2 - height), y2 - max(0, y2 - height)
    x1, x2 = x1 + max(0, -x1), x2 + max(0, -x1)
    y1, y2 = y1 + max(0, -y1), y2 + max(0, -y1)
    if x1 < 0 or y1 < 0 or x2 > width or y2 > height or x2 - x1 < 16 or y2 - y1 < 16:
        return None
    return x1, y1, x2, y2


def box_iou(reference, candidate):
    left, top, right, bottom = reference
    area = (right - left) * (bottom - top)
    x1, y1, x2, y2 = candidate
    overlap = max(0, min(right, x2) - max(left, x1)) * max(0, min(bottom, y2) - max(top, y1))
    candidate_area = (x2 - x1) * (y2 - y1)
    return overlap / max(1, area + candidate_area - overlap)


def requested_color(description):
    """Only constrain goals that explicitly name a supported chromatic color."""
    return next((name for name in CHROMATIC_GOALS if name in description), None)


def box_matches_requested_color(image, box, description):
    """Reject a VLM box or tracker handoff with the wrong dominant object color."""
    color = requested_color(description)
    if color is None:
        return True
    import cv2
    import numpy as np

    if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
        return False
    x1, y1, x2, y2 = box
    width, height = x2 - x1, y2 - y1
    # Use the inside of the proposed object, not the background around its box.
    inset_x, inset_y = width // 8, height // 8
    left, top = max(0, x1 + inset_x), max(0, y1 + inset_y)
    right = min(image.shape[1], x2 - inset_x)
    bottom = min(image.shape[0], y2 - inset_y)
    if right - left < 8 or bottom - top < 8:
        return False
    crop = image[top:bottom, left:right]
    if max(crop.shape[:2]) > 96:
        crop = cv2.resize(crop, (min(96, crop.shape[1]), min(96, crop.shape[0])),
                          interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    vivid = (saturation >= 70) & (value >= 45)
    vivid_count = int(np.count_nonzero(vivid))
    if vivid_count < max(12, round(hue.size * 0.10)):
        return False
    def pixels_for(name):
        return sum(int(np.count_nonzero(vivid & (hue >= low) & (hue <= high)))
                   for low, high in CHROMATIC_GOALS[name])
    wanted = pixels_for(color)
    competitors = max(pixels_for(name) for name in CHROMATIC_GOALS if name != color)
    return wanted >= max(12, round(hue.size * 0.10), round(vivid_count * 0.35)) and wanted > competitors


def parse_visual_goal(text: str) -> VisualGoal | None:
    """Accept only search/approach intents; never turn arbitrary prose into motion."""
    value = "".join(text.strip().split()).rstrip("。！!，,")
    if not 2 <= len(value) <= 200 or any(word in value for word in ("不要", "别", "禁止", "停止")):
        return None
    if re.fullmatch(r'[红黄绿蓝](?:色)?(?:的)?(?:按摩小球|按摩球|刺球|小球|球)', value):
        return VisualGoal(value, False)  # A bare color/name requests search, never approach.
    thermal_check = False
    suffix = re.search(r"[，,、]?(?:并|然后|再)(?:实行|施行|实施|执行|进行|开启)?(?:温度检测|温度监测|测温|热成像检测|检测温度|(?:检查|判断)(?:是否)?(?:异常发热|发热异常|过热|发热))(?:[，,、]*(?:并|再)?(?:判断|检查|检测)(?:是否)?(?:异常发热|发热异常|过热|发热))?$", value)
    if suffix:
        value = value[:suffix.start()].rstrip('，,、')
        thermal_check = True
    match = re.fullmatch(
        r"(?:请帮我|帮我|请)?(?:寻找|找到|搜索|识别|找)(.+?)"
        r"((?:[，,]?(?:并|然后))(?:自主)?(?:移动到(?:它|其)?(?:附近|前面)|靠近(?:它|其)?|接近(?:它|其)?))?", value)
    if not match:
        return None
    description = match.group(1).removesuffix("的")
    if (description in ("", "目标", "物体", "东西", "未知类别物体")
            or len(description) > 48 or "并" in description or "附近" in description):
        return None
    approach = bool(match.group(2))
    if thermal_check and not approach:
        return None
    return VisualGoal(description, approach, thermal_check)


def parse_vlm_boxes(answer: str, width: int, height: int, fallback="object") -> list[tuple[int, int, int, int]]:
    """Require model-produced normalized JSON boxes; reject malformed geometry."""
    answer = answer.strip()
    if answer.startswith("```"):
        answer = re.sub(r"^```(?:json)?\s*|\s*```$", "", answer, flags=re.IGNORECASE)
    try:
        items = json.loads(answer)
    except (ValueError, TypeError):
        return []
    if not isinstance(items, list) or len(items) > 8:
        return []
    result = VlmBoxes()
    for item in items:
        coords = item.get("bbox") if isinstance(item, dict) else None
        if not isinstance(coords, list) or len(coords) != 4 or any(type(v) not in (int, float) for v in coords):
            continue
        x1, y1, x2, y2 = coords
        if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
            continue
        box = (round(x1 * width), round(y1 * height), round(x2 * width), round(y2 * height))
        if box[2] - box[0] >= 16 and box[3] - box[1] >= 16:
            result.append(box)
            name=next((item.get(key) for key in ('label','english_name','name','class_name')
                       if isinstance(item.get(key),str) and item.get(key).strip()),None)
            result.labels[box]=clean_english_label(name,fallback)
    return result


class VlmClient:
    def __init__(self, url: str, model: str = "task6-vlm", timeout: float = 30.0):
        if not url.startswith("http://127.0.0.1:") and not url.startswith("http://localhost:"):
            raise ValueError("VLM endpoint must be on the Raspberry Pi loopback interface")
        self.url = url
        self.model = model
        self.timeout = timeout

    def locate(self, image, description: str):
        import cv2

        ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            raise RuntimeError("Cannot encode camera frame for VLM")
        image_url = "data:image/jpeg;base64," + base64.b64encode(jpeg.tobytes()).decode("ascii")
        prompt = (f"Find every visible instance matching exactly this description: {description}. "
                  "Return ONLY a JSON array of objects with label and bbox fields. "
                  "label must be a specific short English object name, including its requested color, "
                  "using at most 15 printable ASCII characters (for example red cup). "
                  "Use the object's English noun, never target, VLM target, or unknown. "
                  "bbox must be [x1,y1,x2,y2] normalized to [0,1]. "
                  "If none is visible, return []. Do not guess boxes.")
        payload = {"model": self.model, "messages": [{"role": "user", "content": [
            {"type": "text", "text": prompt},
            {"type": "image_url", "image_url": {"url": image_url}},
        ]}], "temperature": 0, "max_tokens": 160, "stream": False}
        request = urllib.request.Request(self.url, json.dumps(payload).encode("utf-8"),
                                         {"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            data = json.load(response)
        answer = data["choices"][0]["message"]["content"]
        if not isinstance(answer, str):
            raise ValueError("VLM returned non-text content")
        return parse_vlm_boxes(answer, image.shape[1], image.shape[0],english_goal_label(description))


class VlmWorker(threading.Thread):
    def __init__(self, client):
        super().__init__(name="vlm-inference", daemon=True)
        self.client = client
        self.requests = queue.Queue(maxsize=1)
        self.results = queue.Queue(maxsize=1)
        self.start()

    def submit(self, generation, image, description):
        try:
            self.requests.put_nowait((generation, image.copy(), description))
            return True
        except queue.Full:
            return False

    def run(self):
        while True:
            generation, image, description = self.requests.get()
            try:
                boxes = self.client.locate(image, description)
                result = (generation, image, boxes, None)
            except Exception as error:
                result = (generation, image, [], str(error))
            try:
                self.results.put_nowait(result)
            except queue.Full:
                self.results.get_nowait()
                self.results.put_nowait(result)

    def poll(self):
        try:
            return self.results.get_nowait()
        except queue.Empty:
            return None


class TemplateTracker:
    """Track a target's interior over bounded shifts and scale changes."""

    def __init__(self, image, box):
        import cv2

        self.cv2 = cv2
        self.box = box
        self.base_width = box[2] - box[0]
        self.base_height = box[3] - box[1]
        self.margin_x = min(self.base_width // 6, (self.base_width - 16) // 2)
        self.margin_y = min(self.base_height // 6, (self.base_height - 16) // 2)
        self.scale = 1.0
        color = image[box[1] + self.margin_y:box[3] - self.margin_y,
                      box[0] + self.margin_x:box[2] - self.margin_x]
        if color.shape[0] < 16 or color.shape[1] < 16:
            raise ValueError("VLM target crop is too small")
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        self.low_texture = gray.std() < 8
        if self.low_texture:
            # A uniform interior has no position cue; keep its object boundary.
            self.margin_x = self.margin_y = 0
            self.template = image[box[1]:box[3], box[0]:box[2]].copy()
        else:
            self.template = gray

    def update(self, image):
        import numpy as np

        x1, y1, x2, y2 = self.box
        pad_x = max(32, (x2 - x1) // 2)
        pad_y = max(24, (y2 - y1) // 2)
        left, top = max(0, x1 - pad_x), max(0, y1 - pad_y)
        right = min(image.shape[1], x2 + pad_x)
        bottom = min(image.shape[0], y2 + pad_y)
        region = image[top:bottom, left:right]
        search = region if self.low_texture else self.cv2.cvtColor(region, self.cv2.COLOR_BGR2GRAY)
        best = None
        for factor in (0.85, 1.0, 1.17, 1.4):
            scale = self.scale * factor
            width = round(self.template.shape[1] * scale)
            height = round(self.template.shape[0] * scale)
            if width < 16 or height < 16 or width > search.shape[1] or height > search.shape[0]:
                continue
            template = self.cv2.resize(self.template, (width, height))
            method = self.cv2.TM_SQDIFF_NORMED if self.low_texture else self.cv2.TM_CCOEFF_NORMED
            scores = self.cv2.matchTemplate(search, template, method)
            minimum, maximum, minimum_at, maximum_at = self.cv2.minMaxLoc(scores)
            quality = 1 - minimum if self.low_texture else maximum
            if not np.isfinite(quality) or quality < (0.92 if self.low_texture else 0.78):
                continue
            location = minimum_at if self.low_texture else maximum_at
            # Prefer a steady scale when the image supports two similar sizes.
            adjusted = quality - 0.04 * abs(factor - 1)
            if best is None or adjusted > best[0]:
                best = adjusted, scale, location
        if best is None:
            return None
        _, scale, location = best
        nx = left + location[0] - round(self.margin_x * scale)
        ny = top + location[1] - round(self.margin_y * scale)
        full_width = round(self.base_width * scale)
        full_height = round(self.base_height * scale)
        clipped = (max(0, nx), max(0, ny),
                   min(image.shape[1], nx + full_width),
                   min(image.shape[0], ny + full_height))
        if clipped[2] - clipped[0] < 16 or clipped[3] - clipped[1] < 16:
            return None
        self.scale = scale
        self.box = clipped
        return self.box


class VlmTaskController:
    """Stop/scan/query/verify/track. FPGA owns steering and 30 cm stopping."""

    def __init__(self, send_command, client, clock=time.monotonic,
                 tracker_factory=TemplateTracker, require_yolo_assist=False):
        self.send = send_command
        self.worker = VlmWorker(client)
        self.clock = clock
        self.tracker_factory = tracker_factory
        self.require_yolo_assist = require_yolo_assist
        self.generation = 0
        self.session = secrets.randbits(32) or 1
        self.track_id = secrets.randbelow(1 << 24) + 1
        self.phase = "idle"
        self.goal = None
        self.english_label = "object"
        self.last_error = ""
        self.scans = 0
        self.turn_until = 0.0
        self.search_speed_boosted = False
        self.settle_until = 0.0
        self.last_frame = 0.0
        self.track_started = 0.0
        self.tracker = None
        self.published = 0
        self.select_started = 0.0
        self.saw_lock = False
        self.lock_confirmed_at = None
        self.lock_missing_frames = 0
        self.approach_started = 0.0
        self.last_box = None
        self.recheck_pending = False
        self.recheck_started = 0.0
        self.recheck_box = None
        self.yolo_track_id = None
        self.yolo_session = None
        self.last_yolo_frame_id = None
        self.last_yolo_seen_at = 0.0
        self.last_progress_log = 0.0

    @property
    def active(self):
        return self.phase not in ("idle", "stopped", "failed", "arrival_pending", "thermal")

    def status(self):
        return {"phase": self.phase, "target": self.goal.description if self.goal else "", "english_label":self.english_label,
                "approach": self.goal.approach if self.goal else False,
                "scans": self.scans, "bbox": self.last_box, "message": self.last_error}

    def submit(self, goal: VisualGoal):
        self.cancel()
        self.send(MOTION["停车"])
        self.goal = goal
        self.english_label=english_goal_label(goal.description)
        self.generation += 1
        self.session = secrets.randbits(32) or 1
        self.track_id = secrets.randbelow((1 << 24) - 1) + 1
        self.phase = "search"
        self.last_error = ""
        self.scans = 0
        self.last_box = None
        self.recheck_pending = False
        self.recheck_box = None
        self.yolo_track_id = None
        self.yolo_session = None
        self.last_yolo_frame_id = None
        self.last_yolo_seen_at = 0.0
        self.last_progress_log = 0.0
        self.saw_lock = False
        self.lock_confirmed_at = None
        self.lock_missing_frames = 0
        self.settle_until = self.clock() + 0.2
        LOG.info("VLM task: find %s, approach=%s", goal.description, goal.approach)

    def cancel(self):
        self.lock_confirmed_at = None
        if self.phase != "idle":
            self.send(MOTION["停车"])
        self._restore_search_speed()
        self.phase = "idle"
        self.generation += 1
        self.tracker = None
        self.goal = None
        self.last_box = None
        self.recheck_pending = False
        self.recheck_box = None
        self.yolo_track_id = None
        self.yolo_session = None
        self.last_yolo_frame_id = None

    def abort(self, reason):
        self.lock_confirmed_at = None
        self.send(MOTION["停车"])
        self._restore_search_speed()
        self.phase = "failed"
        self.last_error = reason
        self.tracker = None
        self.last_box = None
        self.recheck_pending = False
        self.recheck_box = None
        self.yolo_track_id = None
        self.yolo_session = None
        self.last_yolo_frame_id = None

    def _recheck(self, reason):
        self.lock_confirmed_at = None
        self.send(MOTION["停车"])
        self._restore_search_speed()
        LOG.warning("VLM target verification paused motion: %s; phase=%s last_box=%s",
                    reason, self.phase, self.last_box)
        self.generation += 1  # Ignore any reply from an in-flight background recheck.
        self.tracker = None
        self.last_box = None
        self.recheck_pending = False
        self.recheck_box = None
        self.yolo_track_id = None
        self.yolo_session = None
        self.last_yolo_frame_id = None
        self.last_error = reason
        self.settle_until = self.clock() + 0.3
        self.phase = "search"

    def _retry(self, reason):
        self.lock_confirmed_at = None
        self.send(MOTION["停车"])
        self._restore_search_speed()
        self.tracker = None
        self.last_box = None
        self.recheck_pending = False
        self.recheck_box = None
        self.yolo_track_id = None
        self.yolo_session = None
        self.last_yolo_frame_id = None
        self.last_error = reason
        self.scans += 1
        if self.scans >= 12:
            self.phase = "failed"
            return
        self.send(MOTION["加速"])
        self.search_speed_boosted = True
        self.send(MOTION["原地左转"])
        self.turn_until = self.clock() + SEARCH_TURN_SECONDS
        self.phase = "turning"

    def _restore_search_speed(self):
        if self.search_speed_boosted:
            self.send(MOTION["减速"])
            self.search_speed_boosted = False

    def tick(self):
        now = self.clock()
        if self.phase == "turning" and now >= self.turn_until:
            self.send(MOTION["停车"])
            self._restore_search_speed()
            self.settle_until = now + SEARCH_SETTLE_SECONDS
            self.phase = "search"
        if self.phase in ("verify", "select", "approach", "found") and now - self.last_frame > 0.8:
            self.abort("Camera frames stopped")

    def observe_lock(self, lock):
        if self.phase != "approach":
            return
        matching = lock is not None and (lock.tracker_session, lock.track_id) == (self.session, self.track_id)
        if matching:
            if self.lock_confirmed_at is None:
                self.lock_confirmed_at = self.clock()
            if not self.saw_lock:
                LOG.info("FPGA confirmed VLM lock session=%08x id=%d", self.session, self.track_id)
            self.saw_lock = True
            self.lock_missing_frames = 0
        elif self.saw_lock:
            self.lock_confirmed_at = None
            self.lock_missing_frames += 1
            if self.lock_missing_frames >= 3:
                self.send(MOTION["停车"])
                self.phase = "arrival_pending" if self.goal and self.goal.thermal_check else "stopped"
                self.arrival_pending_at = self.clock()
                self.last_error = "FPGA 已释放目标锁定；可能已到 30 cm 附近或因安全条件停车，需核对实际距离"
        elif self.clock() - self.approach_started > 1.5:
            self.send(MOTION["停车"])
            self.phase = "arrival_pending" if self.goal and self.goal.thermal_check else "failed"
            self.arrival_pending_at = self.clock()
            self.last_error = "FPGA 未回报目标锁定；目标可能已在 30 cm 内，也可能是目标框或控制指令未被接收"
            LOG.warning("FPGA did not confirm VLM lock session=%08x id=%d; last feedback=%s",
                        self.session, self.track_id, lock)

    def _yolo_reseed(self, image, observation, now):
        if (observation is None or self.goal is None or
                (target_class := yolo_class_for_goal(self.goal.description)) is None or
                observation.frame_id == self.last_yolo_frame_id or
                not 0 <= now - observation.captured_at <= 0.25 or
                self.last_box is None):
            return None
        self.last_yolo_frame_id = observation.frame_id
        candidates = [box for box in observation.boxes
                      if box.class_id == target_class and box.track_id and
                      box.flags & OBSERVED and box.confidence >= 350 and
                      box_matches_requested_color(observation.image,
                                                  (box.x1, box.y1, box.x2, box.y2),
                                                  self.goal.description)]
        if self.yolo_track_id is not None:
            candidates = [box for box in candidates
                          if box.track_id == self.yolo_track_id and
                          observation.tracker_session == self.yolo_session]
        if not candidates:
            return None
        chosen = max(candidates,
                     key=lambda box: box_iou(self.last_box, (box.x1, box.y1, box.x2, box.y2)))
        overlap = box_iou(self.last_box, (chosen.x1, chosen.y1, chosen.x2, chosen.y2))
        if overlap < (0.12 if self.yolo_track_id is not None else 0.25):
            if self.yolo_track_id is not None:
                self._recheck("YOLO target moved away from the VLM target")
                return False
            return None
        try:
            tracker = self.tracker_factory(observation.image,
                                           (chosen.x1, chosen.y1, chosen.x2, chosen.y2))
            current_box = tracker.update(image)
        except ValueError:
            return None
        if current_box is None:
            return None
        if self.yolo_track_id is None:
            LOG.info("VLM target paired with YOLO class=%d track=%d",
                     target_class, chosen.track_id)
        self.tracker = tracker
        self.yolo_track_id = chosen.track_id
        self.yolo_session = observation.tracker_session
        self.last_yolo_seen_at = now
        return current_box

    def process(self, image, frame_id, yolo_observation=None):
        self.tick()
        now = self.clock()
        self.last_frame = now
        if self.phase == "search":
            if now >= self.settle_until and self.worker.submit(self.generation, image, self.goal.description):
                self.phase = "querying"
            return []
        if self.phase == "querying":
            result = self.worker.poll()
            if result is None:
                return []
            generation, source, boxes, error = result
            labels=getattr(boxes,'labels',{})
            if generation != self.generation:
                return []
            if error:
                self.abort("VLM request failed: " + error)
                return []
            proposed_boxes = len(boxes)
            if requested_color(self.goal.description):
                boxes = [box for box in boxes if box_matches_requested_color(
                    source, box, self.goal.description)]
                if proposed_boxes and not boxes:
                    LOG.warning("VLM proposed %d box(es), none matches requested color %s",
                                proposed_boxes, requested_color(self.goal.description))
            if not boxes:
                self._retry("Requested color not visible" if proposed_boxes else "Target not visible")
                return []
            LOG.info("VLM found %s: %s", self.goal.description, boxes)
            self.english_label=labels.get(boxes[0],self.english_label)
            self.last_error = ""
            try:
                self.tracker = self.tracker_factory(source, boxes[0])
            except ValueError as failure:
                self._retry(str(failure))
                return []
            self.phase = "verify"
            self.track_started = now
            self.published = 0
            self.select_started = 0.0
        if self.phase in ("verify", "select", "approach", "found"):
            if self.phase != "approach" and now - self.track_started > ACQUISITION_RECHECK_INTERVAL:
                self._recheck("Rechecking target with VLM")
                return []
            reseeded = (self._yolo_reseed(image, yolo_observation, now)
                        if self.require_yolo_assist and self.phase in ("select", "approach") else None)
            if reseeded is False:
                return []
            box = reseeded if reseeded is not None else self.tracker.update(image)
            if box is None:
                self._recheck("Visual target lost")
                return []
            if not box_matches_requested_color(image, box, self.goal.description):
                self._recheck("Requested target color no longer visible")
                return []
            if (self.phase == "approach" and self.yolo_track_id is not None and
                    now - self.last_yolo_seen_at > 1.0):
                self._recheck("Paired YOLO target has not been refreshed")
                return []
            if self.phase == "approach":
                if self.recheck_pending:
                    result = self.worker.poll()
                    if result is not None:
                        generation, source, boxes, error = result
                        labels=getattr(boxes,'labels',{})
                        if generation == self.generation:
                            self.recheck_pending = False
                            if requested_color(self.goal.description):
                                boxes = [candidate for candidate in boxes
                                         if box_matches_requested_color(
                                             source, candidate, self.goal.description)]
                            if error or not matching_recheck_box(self.recheck_box, boxes):
                                self._recheck("VLM recheck failed: " + error if error else
                                              "VLM recheck did not confirm the target")
                                return []
                            selected = max(boxes, key=lambda candidate:
                                           box_iou(self.recheck_box, candidate))
                            self.english_label=labels.get(selected,self.english_label)
                            corrected = current_recheck_box(
                                self.recheck_box, box, selected, image.shape[1], image.shape[0])
                            if corrected is None or box_iou(box, corrected) < 0.20:
                                self._recheck("VLM update disagreed with current target")
                                return []
                            try:
                                self.tracker = self.tracker_factory(image, corrected)
                            except ValueError:
                                self._recheck("VLM update could not refresh tracker")
                                return []
                            box = corrected
                            self.track_started = now
                            self.last_error = ""
                            LOG.info("VLM updated moving target box to %s", corrected)
                    if self.recheck_pending and now - self.recheck_started > RECHECK_TIMEOUT:
                        self._recheck("VLM background recheck timed out")
                        return []
                elif now - self.track_started >= RECHECK_INTERVAL:
                    if self.worker.submit(self.generation, image, self.goal.description):
                        self.recheck_pending = True
                        self.recheck_started = now
                        self.recheck_box = box
                        LOG.info("VLM background recheck started while approaching")
                    elif now - self.track_started > RECHECK_INTERVAL + RECHECK_TIMEOUT:
                        self._recheck("VLM background recheck could not start")
                        return []
            if self.phase == "verify" and not self.goal.approach:
                self.send(MOTION["停车"])
                self.phase = "found"
            self.last_box = box
            self.phase = "select" if self.phase == "verify" else self.phase
            if self.phase == "approach" and now - self.last_progress_log >= 2.0:
                LOG.info("VLM approach frame=%d box=%s recheck_pending=%s",
                         frame_id, box, self.recheck_pending)
                self.last_progress_log = now
            return [DetectionBox(*box, 700, VLM_WIRE_CLASS, self.track_id, OBSERVED)]
        return []

    def after_sent(self):
        if self.phase == "select":
            self.published += 1
            if self.published == 1:
                self.send(0x46)
                self.select_started = self.clock()
                LOG.info("VLM target selection requested; waiting for FPGA box commit")
            elif self.published >= 4 and self.clock() - self.select_started >= 0.15:
                if (self.require_yolo_assist and self.goal is not None and
                        yolo_class_for_goal(self.goal.description) is not None and
                        self.yolo_track_id is None):
                    if self.clock() - self.select_started > 1.5:
                        self._recheck("YOLO did not confirm the VLM target before approach")
                    return
                # FBOX reaches the FPGA through an Ethernet parser and a
                # clock-domain mailbox.  Confirm only after several fresh
                # measured boxes have been published in SELECT mode.
                self.send(0xA1)
                self.phase = "approach"
                self.lock_confirmed_at = None
                self.approach_started = self.clock()
                LOG.info("VLM approach confirmation sent after %d target frames", self.published)
