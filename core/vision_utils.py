"""
KP2026 Vision Utilities — letterbox, YOLO decoding, NMS, output format detection.

Single source of truth shared by HailoDetector in core/ and tools/.
"""

import re
from typing import Dict, List, Tuple

import cv2
import numpy as np

from config import NMS_IOU_THRESHOLD, NUM_CLASSES


def sigmoid(x: np.ndarray) -> np.ndarray:
    x_c = np.clip(x, -20, 20)
    return 1.0 / (1.0 + np.exp(-x_c))


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


# ═══════════════════════════════════════════════════════════════════════════
# Preprocessing
# ═══════════════════════════════════════════════════════════════════════════


def letterbox_bgr_to_rgb_uint8(
    frame_bgr: np.ndarray, input_w: int, input_h: int
) -> Tuple[np.ndarray, float, int, int]:
    """Resize + letterbox preserving aspect ratio, BGR→RGB."""
    orig_h, orig_w = frame_bgr.shape[:2]
    scale = min(input_w / orig_w, input_h / orig_h)
    new_w = int(round(orig_w * scale))
    new_h = int(round(orig_h * scale))
    resized = cv2.resize(frame_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((input_h, input_w, 3), 114, dtype=np.uint8)
    pad_x = (input_w - new_w) // 2
    pad_y = (input_h - new_h) // 2
    canvas[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = resized
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    return rgb, scale, pad_x, pad_y


def map_bbox_back(
    x1: float, y1: float, x2: float, y2: float,
    scale: float, pad_x: int, pad_y: int,
    orig_w: int, orig_h: int,
) -> Tuple[int, int, int, int]:
    """Map bounding box from letterbox coordinates to original frame."""
    nx1 = (x1 - pad_x) / scale
    nx2 = (x2 - pad_x) / scale
    ny1 = (y1 - pad_y) / scale
    ny2 = (y2 - pad_y) / scale
    return (
        int(max(0, min(orig_w - 1, nx1))),
        int(max(0, min(orig_h - 1, ny1))),
        int(max(0, min(orig_w - 1, nx2))),
        int(max(0, min(orig_h - 1, ny2))),
    )


# ═══════════════════════════════════════════════════════════════════════════
# NMS
# ═══════════════════════════════════════════════════════════════════════════


def fast_nms(
    boxes: List[List[float]], scores: List[float], class_ids: List[int],
    iou_thres: float = NMS_IOU_THRESHOLD,
) -> Tuple[List, List, List]:
    """Fast NMS for raw YOLO outputs."""
    if len(boxes) == 0:
        return [], [], []
    boxes_arr = np.array(boxes, dtype=np.float32)
    scores_arr = np.array(scores, dtype=np.float32)
    x1 = boxes_arr[:, 0]
    y1 = boxes_arr[:, 1]
    x2 = boxes_arr[:, 2]
    y2 = boxes_arr[:, 3]
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores_arr.argsort()[::-1]
    keep: List[int] = []
    while len(order) > 0:
        i = order[0]
        keep.append(i)
        if len(order) == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= iou_thres)[0]
        order = order[inds + 1]
    return (
        [boxes[i] for i in keep],
        [scores[i] for i in keep],
        [class_ids[i] for i in keep],
    )


# ═══════════════════════════════════════════════════════════════════════════
# Output format detection
# ═══════════════════════════════════════════════════════════════════════════


def detect_output_format(outputs: Dict) -> str:
    """Auto-detect: 'raw_yolo' or 'nms'."""
    has_raw_box = False
    has_nms = False
    for out_arr in outputs.values():
        arr = np.array(out_arr)
        if arr.ndim == 4 and arr.shape[0] == 1:
            arr = arr[0]
        if arr.ndim != 3:
            continue
        C = arr.shape[-1]
        if C == 4:
            has_raw_box = True
        if C >= 5:
            has_nms = True
    if has_raw_box and not has_nms:
        return "raw_yolo"
    if has_nms:
        return "nms"
    return "raw_yolo"


# ═══════════════════════════════════════════════════════════════════════════
# Name-based matching (box ↔ score pairing)
# ═══════════════════════════════════════════════════════════════════════════


def match_box_score_by_name(
    outputs: Dict, input_w: int, num_classes: int = NUM_CLASSES
) -> List[Tuple[np.ndarray, np.ndarray, int]]:
    """Fallback: match box↔score by layer name convention."""
    pairs: List[Tuple[np.ndarray, np.ndarray, int]] = []
    box_candidates: Dict[int, Tuple[np.ndarray, int]] = {}
    score_candidates: Dict[int, Tuple[np.ndarray, int]] = {}

    for out_arr in outputs.values():
        arr = np.array(out_arr)
        if arr.ndim == 4 and arr.shape[0] == 1:
            arr = arr[0]
        if arr.ndim != 3:
            continue
        H, W, C = arr.shape
        stride = input_w // W
        # Extract layer number from name keys isn't possible here —
        # callers should pass name→array dict if needed.
        if C == 4:
            box_candidates[stride] = (arr, stride)
        elif C == num_classes:
            score_candidates[stride] = (arr, stride)

    for stride in sorted(box_candidates.keys()):
        if stride in score_candidates:
            pairs.append((box_candidates[stride][0], score_candidates[stride][0], stride))
    return pairs


# ═══════════════════════════════════════════════════════════════════════════
# Raw YOLO decoder
# ═══════════════════════════════════════════════════════════════════════════


def decode_raw_yolo(
    outputs: Dict,
    conf_thres: float,
    input_w: int,
    input_h: int,
    scale: float,
    pad_x: int,
    pad_y: int,
    orig_w: int,
    orig_h: int,
    num_classes: int = NUM_CLASSES,
) -> List[Dict]:
    """Decoder for raw YOLO head outputs (box + score per scale → NMS → map back)."""
    box_outputs: Dict[int, np.ndarray] = {}
    score_outputs: Dict[int, np.ndarray] = {}

    for out_name, out_arr in outputs.items():
        arr = np.array(out_arr)
        if arr.ndim == 4 and arr.shape[0] == 1:
            arr = arr[0]
        if arr.ndim != 3:
            continue
        H, W, C = arr.shape
        stride = input_w // W
        if C == 4 and H >= 20:
            box_outputs[stride] = arr
        elif C == num_classes and H >= 20:
            score_outputs[stride] = arr

    # Pair by matching strides
    pairs = []
    for stride in sorted(box_outputs.keys()):
        if stride in score_outputs:
            pairs.append((box_outputs[stride], score_outputs[stride], stride))

    if not pairs:
        pairs = _match_by_name(outputs, input_w, num_classes)

    if not pairs:
        return []

    all_boxes: List[List[float]] = []
    all_scores: List[float] = []
    all_class_ids: List[int] = []

    for box_arr, score_arr, stride in pairs:
        H, W, _ = box_arr.shape
        scores_sigmoid = sigmoid(score_arr)
        max_scores = np.max(scores_sigmoid, axis=2)
        max_class_ids = np.argmax(scores_sigmoid, axis=2)
        ys, xs = np.where(max_scores > conf_thres)
        for y, x in zip(ys, xs):
            conf = float(max_scores[y, x])
            class_id = int(max_class_ids[y, x])
            left, top, right, bottom = box_arr[y, x, :]
            cx_g = (x + 0.5) * stride
            cy_g = (y + 0.5) * stride
            all_boxes.append([
                cx_g - left * stride,
                cy_g - top * stride,
                cx_g + right * stride,
                cy_g + bottom * stride,
            ])
            all_scores.append(conf)
            all_class_ids.append(class_id)

    if not all_boxes:
        return []

    all_boxes, all_scores, all_class_ids = fast_nms(all_boxes, all_scores, all_class_ids)

    detections: List[Dict] = []
    for i in range(len(all_boxes)):
        x1, y1, x2, y2 = all_boxes[i]
        score = all_scores[i]
        class_id = all_class_ids[i]
        x1m, y1m, x2m, y2m = map_bbox_back(
            x1, y1, x2, y2, scale, pad_x, pad_y, orig_w, orig_h
        )
        if x2m <= x1m or y2m <= y1m:
            continue
        detections.append({
            "class_id": class_id,
            "score": score,
            "bbox": (x1m, y1m, x2m, y2m),
        })
    return detections


def _match_by_name(
    outputs: Dict, input_w: int, num_classes: int = NUM_CLASSES
) -> List[Tuple[np.ndarray, np.ndarray, int]]:
    """Fallback: match box↔score by layer number convention (conv61↔conv64, etc)."""
    box_candidates: Dict[int, Tuple[np.ndarray, int]] = {}
    score_candidates: Dict[int, Tuple[np.ndarray, int]] = {}

    for name, out_arr in outputs.items():
        arr = np.array(out_arr)
        if arr.ndim == 4 and arr.shape[0] == 1:
            arr = arr[0]
        if arr.ndim != 3:
            continue
        H, W, C = arr.shape
        stride = input_w // W
        nums = re.findall(r"\d+", name)
        layer_num = int(nums[-1]) if nums else 0
        if C == 4:
            box_candidates[layer_num] = (arr, stride)
        elif C == num_classes:
            score_candidates[layer_num] = (arr, stride)

    pairs: List[Tuple[np.ndarray, np.ndarray, int]] = []
    for bnum, (barr, bstride) in box_candidates.items():
        for snum, (sarr, sstride) in score_candidates.items():
            if snum == bnum + 3 and bstride == sstride:
                pairs.append((barr, sarr, bstride))
                break
    return pairs


# ═══════════════════════════════════════════════════════════════════════════
# NMS-Processed output parser
# ═══════════════════════════════════════════════════════════════════════════


def parse_nms_outputs(
    outputs: Dict,
    conf_thres: float,
    input_w: int,
    input_h: int,
    scale: float,
    pad_x: int,
    pad_y: int,
    orig_w: int,
    orig_h: int,
) -> List[Dict]:
    """Parser for Hailo NMS-processed outputs."""
    detections: List[Dict] = []

    for out_arr in outputs.values():
        arr = np.array(out_arr)
        if arr.ndim == 4 and arr.shape[0] == 1:
            arr = arr[0]

        if arr.ndim == 3 and arr.shape[-1] >= 5:
            num_cls = arr.shape[0]
            for cls_id in range(num_cls):
                for det in arr[cls_id]:
                    ymin, xmin, ymax, xmax, score = map(float, det[:5])
                    if score < conf_thres:
                        continue
                    if max(xmin, ymin, xmax, ymax) <= 1.5:
                        xmin *= input_w
                        ymin *= input_h
                        xmax *= input_w
                        ymax *= input_h
                    x1, y1, x2, y2 = map_bbox_back(
                        xmin, ymin, xmax, ymax, scale, pad_x, pad_y, orig_w, orig_h
                    )
                    if x2 <= x1 or y2 <= y1:
                        continue
                    detections.append({
                        "class_id": int(cls_id),
                        "score": score,
                        "bbox": (x1, y1, x2, y2),
                    })

        elif arr.ndim in [2, 3] and arr.shape[-1] >= 6:
            if arr.ndim == 3 and arr.shape[0] == 1:
                arr = arr[0]
            for det in arr:
                x1, y1, x2, y2, score, cls_id = map(float, det[:6])
                if score < conf_thres:
                    continue
                if max(x1, y1, x2, y2) <= 1.5:
                    x1 *= input_w
                    y1 *= input_h
                    x2 *= input_w
                    y2 *= input_h
                x1, y1, x2, y2 = map_bbox_back(
                    x1, y1, x2, y2, scale, pad_x, pad_y, orig_w, orig_h
                )
                if x2 <= x1 or y2 <= y1:
                    continue
                detections.append({
                    "class_id": int(cls_id),
                    "score": score,
                    "bbox": (int(x1), int(y1), int(x2), int(y2)),
                })

    return detections
