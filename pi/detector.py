#!/usr/bin/env python3
"""
detector.py - Hailo-8 YOLOv26 object detector (dummy pipeline).

Loads a compiled .hef model, runs inference on camera frames and draws
bounding boxes. This is intentionally a *dummy* stage: detection results are
only drawn on the frame, no action is taken from them (no control, no
telemetry, no logging pipeline).

Used by camera_server.py to overlay detections on the MJPEG stream.

Model layout (KP2026V1-YOLOv26.hef, verified via HEF metadata):
    input : yolo26n/input_layer1  (640, 640, 3)  RGB uint8
    outputs (3 scales, box + cls per scale):
        conv61/77/91  (80|40|20, W, 4)  box logits
        conv64/80/94  (80|40|20, W, 3)  class logits (gate, container, waypoint)
"""

import os

import cv2
import numpy as np

# Class index order follows the training order of KP2026V1-YOLOv26
# (verified empirically: gates come out as class 1).
CLASSES = ["container", "gate", "waypoint"]

INPUT_SIZE = 640
STRIDES = (8, 16, 32)
CONF_THRESH = 0.25
IOU_THRESH = 0.45


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


class Detector:
    """YOLOv26 detector running on the Hailo-8, thread-safe per call."""

    def __init__(self, hef_path, conf_thresh=CONF_THRESH, iou_thresh=IOU_THRESH):
        if not os.path.isfile(hef_path):
            raise FileNotFoundError("HEF model not found: %s" % hef_path)

        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh

        from hailo_platform import (  # heavy import, keep at init
            FormatType,
            HailoSchedulingAlgorithm,
            VDevice,
        )

        params = VDevice.create_params()
        params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        self.vdevice = VDevice(params)
        self.infer_model = self.vdevice.create_infer_model(hef_path)
        self.infer_model.set_batch_size(1)

        # Dequantize outputs to float32 so the decode works on real values.
        for o in self.infer_model.outputs:
            o.set_format_type(FormatType.FLOAT32)

        self.configured = self.infer_model.configure()
        self.configured.set_scheduler_timeout(1000)

        output_names = [o.name for o in self.infer_model.outputs]
        print("Detector model outputs: %s" % output_names)
        self._box_names = [n for n in output_names if n.endswith(("61", "77", "91"))]
        self._cls_names = [n for n in output_names if n.endswith(("64", "80", "94"))]
        if len(self._box_names) != 3 or len(self._cls_names) != 3:
            raise RuntimeError(
                "Unexpected output layout for model %s: %s" % (hef_path, output_names)
            )

        self.bindings = self.configured.create_bindings()
        for o in self.infer_model.outputs:
            self.bindings.output(o.name).set_buffer(
                np.zeros(o.shape, dtype=np.float32)
            )
        print("Detector ready (Hailo-8, %s)" % os.path.basename(hef_path))

    def close(self):
        try:
            self.configured.shutdown()
        except Exception:
            pass
        try:
            self.vdevice.release()
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # Preprocessing: the model was trained on square images via plain
    # resize (no letterbox padding), so we stretch the frame to 640x640.
    # Returns (model_input, sx, sy) where sx=640/frame_w, sy=640/frame_h.
    # ------------------------------------------------------------------ #
    @staticmethod
    def _preprocess(frame):
        h, w = frame.shape[:2]
        resized = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE),
                             interpolation=cv2.INTER_LINEAR)
        return resized, INPUT_SIZE / w, INPUT_SIZE / h

    # ------------------------------------------------------------------ #
    # Postprocessing: YOLOv8-style dist2bbox decode (no DFL, no sigmoid on
    # the box branch). Each cell predicts [lt, rb] distances from the cell
    # anchor center, in grid units:
    #     x1 = (gx + 0.5 - b[0]) * stride   x2 = (gx + 0.5 + b[2]) * stride
    #     y1 = (gy + 0.5 - b[1]) * stride   y2 = (gy + 0.5 + b[3]) * stride
    # ------------------------------------------------------------------ #
    def _decode(self, outputs):
        dets = []  # (x1, y1, x2, y2, conf, class)
        for i, stride in enumerate(STRIDES):
            box = np.asarray(outputs[self._box_names[i]]).astype(np.float32)
            cls = np.asarray(outputs[self._cls_names[i]]).astype(np.float32)
            h, w = cls.shape[:2]

            gx, gy = np.meshgrid(np.arange(w), np.arange(h))
            x1 = (gx + 0.5 - box[..., 0]) * stride
            y1 = (gy + 0.5 - box[..., 1]) * stride
            x2 = (gx + 0.5 + box[..., 2]) * stride
            y2 = (gy + 0.5 + box[..., 3]) * stride

            scores = _sigmoid(cls).reshape(-1, len(CLASSES))
            boxes = np.stack([x1, y1, x2, y2], axis=-1).reshape(-1, 4)

            best_c = scores.argmax(axis=1)
            best_s = scores[np.arange(len(scores)), best_c]
            mask = best_s > self.conf_thresh
            for c in range(len(CLASSES)):
                m = mask & (best_c == c)
                if not m.any():
                    continue
                dets.append(np.column_stack(
                    [boxes[m], best_s[m], np.full(m.sum(), c, dtype=np.int32)]))
        if not dets:
            return []
        dets = np.vstack(dets)
        return self._nms(dets)

    @staticmethod
    def _nms(dets):
        keep = []
        x1, y1, x2, y2, conf, cls = dets.T
        areas = (x2 - x1) * (y2 - y1)
        order = conf.argsort()[::-1]
        while order.size:
            i = order[0]
            keep.append(i)
            if order.size == 1:
                break
            xx1 = np.maximum(x1[order[1:]], x1[i])
            yy1 = np.maximum(y1[order[1:]], y1[i])
            xx2 = np.minimum(x2[order[1:]], x2[i])
            yy2 = np.minimum(y2[order[1:]], y2[i])
            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            union = areas[order[1:]] + areas[i] - inter
            iou = np.where(union > 0, inter / np.maximum(union, 1e-6), 0.0)
            order = order[1:][iou <= 0.45]
        return dets[keep]

    # ------------------------------------------------------------------ #
    # Full pipeline: frame in (BGR) -> list of (x1, y1, x2, y2, conf, cls)
    # in original frame coordinates.
    # ------------------------------------------------------------------ #
    def infer(self, frame):
        h, w = frame.shape[:2]
        canvas, sx, sy = self._preprocess(frame)
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        self.bindings.input().set_buffer(
            np.ascontiguousarray(rgb).reshape(1, INPUT_SIZE, INPUT_SIZE, 3)
        )
        self.configured.run([self.bindings], 10000)

        dets = self._decode({
            name: self.bindings.output(name).get_buffer()
            for name in self._box_names + self._cls_names
        })

        out = []
        for x1, y1, x2, y2, conf, cls_idx in dets:
            out.append((
                int(round(x1 / sx)),
                int(round(y1 / sy)),
                int(round(x2 / sx)),
                int(round(y2 / sy)),
                float(conf),
                int(cls_idx),
            ))
        return out

    # ------------------------------------------------------------------ #
    # Draw detections on a copy of the frame.
    # ------------------------------------------------------------------ #
    COLORS = {
        0: (0, 165, 255),   # gate      -> orange
        1: (0, 255, 0),     # container -> green
        2: (255, 0, 255),   # waypoint  -> magenta
    }

    def annotate(self, frame):
        drawn = frame.copy()
        for x1, y1, x2, y2, conf, cls_idx in self.infer(frame):
            color = self.COLORS.get(cls_idx, (255, 255, 255))
            cv2.rectangle(drawn, (x1, y1), (x2, y2), color, 2)
            label = "%s %.2f" % (CLASSES[cls_idx], conf)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            yt = max(0, y1 - th - 4)
            cv2.rectangle(drawn, (x1, yt), (x1 + tw + 4, y1), color, -1)
            cv2.putText(drawn, label, (x1 + 2, yt + th + 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        return drawn
