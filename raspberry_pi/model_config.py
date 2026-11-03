"""Validate local detector labels and preserve the FPGA's COCO-80 wire IDs."""
from __future__ import annotations

import math
from numbers import Integral

COCO_NAMES = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
)


def model_class_mapping(metadata, manifest=None):
    """Return model-ID -> wire-ID and wire-ID -> label, rejecting ambiguous labels."""
    if not isinstance(metadata, dict) or metadata.get("task") != "detect":
        raise RuntimeError("A detection NCNN model is required")
    if metadata.get("imgsz") != [320, 320]:
        raise RuntimeError("The supplied NCNN graph must have fixed imgsz [320, 320]")
    if metadata.get("batch", 1) != 1 or metadata.get("channels", 3) != 3:
        raise RuntimeError("The NCNN model must use batch 1 and three input channels")
    names = metadata.get("names")
    if isinstance(names, list):
        names = dict(enumerate(names))
    elif isinstance(names, dict):
        converted = {int(k): v for k, v in names.items()
                     if isinstance(k, Integral) or (isinstance(k, str) and k.isdecimal())}
        if len(converted) != len(names):
            raise RuntimeError("Model class IDs must be unique integers")
        names = converted
    else:
        raise RuntimeError("Model metadata must provide class names")
    if not names or set(names) != set(range(len(names))):
        raise RuntimeError("Model class IDs must be contiguous starting at zero")
    if any(not isinstance(name, str) or name not in COCO_NAMES for name in names.values()):
        raise RuntimeError("Every model label must have a supported COCO-80 name")
    mapping = {index: COCO_NAMES.index(name) for index, name in names.items()}
    if len(set(mapping.values())) != len(mapping) or mapping.get(0) != 0:
        raise RuntimeError("Model labels must be unique, with class 0 = person")
    if manifest is not None:
        if not isinstance(manifest, list) or len(manifest) != len(mapping):
            raise RuntimeError("classes.json must describe every model class exactly once")
        seen = set()
        for entry in manifest:
            if not isinstance(entry, dict):
                raise RuntimeError("Invalid classes.json entry")
            index, wire = entry.get("class_id"), entry.get("coco80_id")
            if (type(index) is not int or type(wire) is not int or index in seen
                    or index not in mapping or mapping[index] != wire
                    or entry.get("name") != names[index]):
                raise RuntimeError("classes.json disagrees with model metadata/COCO wire IDs")
            seen.add(index)
    return mapping, {wire: COCO_NAMES[wire] for wire in mapping.values()}


def wire_class_id(value, mapping):
    """Validate an inference label before translating it to a wire ID."""
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if not math.isfinite(value) or not value.is_integer():
        return None
    return mapping.get(int(value))
