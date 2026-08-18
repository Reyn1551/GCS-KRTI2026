"""
KP2026 HailoDetector — unified HailoRT inference wrapper.

Supports both NMS-processed and raw YOLO head outputs. Auto-detects format.
Uses config.py for thresholds and core/vision_utils for preprocessing/decoding.
"""

import logging
from typing import Dict, List, Optional

import numpy as np
from hailo_platform import (
    HEF,
    ConfigureParams,
    FormatType,
    HailoStreamInterface,
    InferVStreams,
    InputVStreamParams,
    OutputVStreamParams,
    VDevice,
)

from config import NMS_IOU_THRESHOLD, NUM_CLASSES
from core.vision_utils import (
    decode_raw_yolo,
    detect_output_format,
    letterbox_bgr_to_rgb_uint8,
    parse_nms_outputs,
)


class HailoDetector:
    """Wrapper for HailoRT inference — compatible with NMS & raw YOLO output."""

    def __init__(self, hef_path: str, conf_thres: float = 0.25):
        self.hef_path = str(hef_path)
        self.conf_thres = conf_thres
        self.hef = HEF(self.hef_path)

        input_infos = self.hef.get_input_vstream_infos()
        if not input_infos:
            raise RuntimeError("HEF has no input vstream info.")
        self.input_name = input_infos[0].name
        input_shape = input_infos[0].shape
        if len(input_shape) == 3:
            self.input_h, self.input_w, self.input_c = input_shape
        else:
            _, self.input_h, self.input_w, self.input_c = input_shape

        logging.info(
            "HailoDetector: %s → %dx%dx%d",
            self.hef_path, self.input_w, self.input_h, self.input_c,
        )

        self.vdevice = VDevice()
        cparams = ConfigureParams.create_from_hef(
            self.hef, interface=HailoStreamInterface.PCIe
        )
        self.network_group = self.vdevice.configure(self.hef, cparams)[0]
        self.network_group_params = self.network_group.create_params()

        self.input_vstream_params = InputVStreamParams.make(
            self.network_group, quantized=True, format_type=FormatType.UINT8
        )
        self.output_vstream_params = OutputVStreamParams.make(
            self.network_group, quantized=False, format_type=FormatType.FLOAT32
        )

        self.activated_ng = self.network_group.activate(self.network_group_params)
        self.activated_ng.__enter__()
        self.infer_pipeline = InferVStreams(
            self.network_group, self.input_vstream_params, self.output_vstream_params
        )
        self.infer_pipeline.__enter__()

        self._output_format: Optional[str] = None

    def close(self):
        try:
            self.infer_pipeline.__exit__(None, None, None)
        except Exception:
            pass
        try:
            self.activated_ng.__exit__(None, None, None)
        except Exception:
            pass

    def infer(self, frame_bgr: np.ndarray) -> List[Dict]:
        """Run inference. Returns list[dict] with keys: class_id, score, bbox."""
        orig_h, orig_w = frame_bgr.shape[:2]
        input_rgb, scale, pad_x, pad_y = letterbox_bgr_to_rgb_uint8(
            frame_bgr, self.input_w, self.input_h
        )
        input_data = {self.input_name: np.expand_dims(input_rgb, axis=0)}
        outputs = self.infer_pipeline.infer(input_data)

        if self._output_format is None:
            self._output_format = detect_output_format(outputs)
            fmt_name = "RAW YOLO" if self._output_format == "raw_yolo" else "NMS"
            logging.info("HailoDetector: detected output format = %s", fmt_name)

        if self._output_format == "raw_yolo":
            return decode_raw_yolo(
                outputs, self.conf_thres,
                self.input_w, self.input_h,
                scale, pad_x, pad_y,
                orig_w, orig_h,
                NUM_CLASSES,
            )
        else:
            return parse_nms_outputs(
                outputs, self.conf_thres,
                self.input_w, self.input_h,
                scale, pad_x, pad_y,
                orig_w, orig_h,
            )
