"""Draw measured boxes on native camera pixels and encode in a bounded worker."""
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
import shutil
import time

from protocol import VIDEO_WIDTH, VIDEO_HEIGHT
from screen_recorder import ScreenWriter


def recognition_view(image):
    """Keep tracker/control coordinates and the fixed320 model preprocessing."""
    import cv2
    if image.shape[:2] == (VIDEO_HEIGHT, VIDEO_WIDTH):
        return image
    return cv2.resize(image, (VIDEO_WIDTH, VIDEO_HEIGHT), interpolation=cv2.INTER_AREA)


def draw_detections(image, boxes, label_for_box, *, work_size=(VIDEO_WIDTH, VIDEO_HEIGHT),
                    selected_track_id=0):
    """Draw onto a COPY: inference/tracking must never see its own overlays."""
    import cv2
    from hsv_balls import BALL_BGR
    from vlm_tasks import clean_english_label, english_goal_label
    result = image.copy()
    height, width = result.shape[:2]
    scale_x, scale_y = width / work_size[0], height / work_size[1]
    thickness = max(1, round(2 * min(scale_x, scale_y)))
    font_scale = 0.45 * min(scale_x, scale_y)
    annotations = []
    for box in boxes:
        left = max(0, min(width - 1, int(box.x1 * scale_x)))
        top = max(0, min(height - 1, int(box.y1 * scale_y)))
        right = max(left + 1, min(width, round(box.x2 * scale_x)))
        bottom = max(top + 1, min(height, round(box.y2 * scale_y)))
        label = str(label_for_box(box))
        # Keep the specific English noun in the video, including legacy callers
        # that still provide the original Chinese task description.
        video_label = clean_english_label(label,english_goal_label(label)) if label.isascii() else english_goal_label(label)
        display_id = box.track_id
        if box.class_id in BALL_BGR and display_id >= 1 << 23:
            display_id -= (1 << 23) - 1
        identity = f' ID:{display_id}' if display_id else ''
        # OpenCV uses BGR. The selected target is blue; other detections retain
        # their existing class colours. Selection comes from FPGA feedback.
        selected = bool(selected_track_id and box.track_id == selected_track_id)
        color = (255, 0, 0) if selected else BALL_BGR.get(box.class_id, (0, 255, 0))
        cv2.rectangle(result, (left, top), (right - 1, bottom - 1), color, thickness)
        text = (f'{video_label}{identity}' if box.class_id in BALL_BGR else
                f'{video_label}{identity} {box.confidence / 1000:.2f}')
        text_width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                                   max(1, round(min(scale_x, scale_y))))[0][0]
        text_left = max(0, min(left, width - text_width - 2))
        cv2.putText(result, text,
                    (text_left, max(round(15 * scale_y), top - round(4 * scale_y))),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, color,
                    max(1, round(min(scale_x, scale_y))))
        annotations.append(dict(asdict(box), label=video_label, original_label=label,
                                native_xyxy=[left, top, right, bottom],selected=selected,
                                box_bgr=list(color)))
    return result, annotations


@dataclass(frozen=True)
class AnnotatedFrame:
    frame_id: int
    width: int
    height: int
    pixels: bytes
    completed_at: float
    sequence_epoch: int
    annotations: tuple
    source: str = "camera"


class ObjectRecorder:
    def __init__(self, output=None, *, fps=30.0, crf=18, threads=2, queue_frames=4):
        ffmpeg = shutil.which('ffmpeg')
        if ffmpeg is None:
            raise RuntimeError('Recording requires FFmpeg: sudo apt install -y ffmpeg')
        self.output = (Path(output).expanduser().resolve() if output is not None else
                       Path(__file__).resolve().parent / 'recordings' /
                       datetime.now().strftime('detections_%Y%m%d_%H%M%S_%f.mkv'))
        self.output.parent.mkdir(parents=True, exist_ok=True)
        for path in [self.output, self.output.with_suffix(self.output.suffix+'.jsonl'),
                     self.output.with_suffix(self.output.suffix+'.ffmpeg.log')]:
            if path.exists():
                raise RuntimeError(f'Refusing to overwrite recording: {path}')
        self.writer = ScreenWriter(self.output, fps=fps, crf=crf, threads=threads,
            queue_frames=queue_frames, ffmpeg=ffmpeg, pixel_format='bgr24',
            input_label='FVID camera with Pi boxes / FREC final HDMI pages; native RGB565; active detector mode',
            frame_metadata=lambda frame: dict(source=frame.source,detections=frame.annotations,
                detection_coordinates=[VIDEO_WIDTH, VIDEO_HEIGHT], unique_detection_fps_is_not_output_fps=True))
        self.submitted = 0
        self.last_submitted_at = None
        self.writer.start()

    @property
    def error(self):
        return self.writer.error

    def submit(self, frame, annotated_image, annotations):
        if self.error:
            return
        height, width = annotated_image.shape[:2]
        annotated_frame = AnnotatedFrame(frame.frame_id, width, height,
            annotated_image.tobytes(), time.perf_counter(), frame.sequence_epoch, tuple(annotations), frame.source)
        self.writer.submit(annotated_frame)
        self.last_submitted_at = annotated_frame.completed_at
        self.submitted += 1

    def close(self):
        # Finish through the last submitted image, including when FFmpeg's
        # startup delayed encoding. Repeats of the first image do not prove
        # that queued camera/HDMI transitions reached the file. This bounded
        # wait occurs AFTER motor STOP/receiver stop.
        deadline = time.monotonic() + 3
        while (self.submitted and
               (self.writer.last_encoded_at is None or self.writer.last_encoded_at < self.last_submitted_at) and
               self.writer.is_alive()
               and not self.error and time.monotonic() < deadline):
            time.sleep(0.01)
        self.writer.stop_event.set()
        self.writer.join(timeout=35)
        if self.writer.is_alive():
            raise RuntimeError('Video encoder did not stop; check its FFmpeg log')
