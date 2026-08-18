"""
KP2026 Camera Utilities — calibration loading, intrinsics.
"""

import json
import logging
from typing import Tuple

from config import FX as DEFAULT_FX
from config import FY as DEFAULT_FY
from config import CX as DEFAULT_CX
from config import CY as DEFAULT_CY


def load_camera_calibration(
    calib_path: str,
) -> Tuple[float, float, float, float, list]:
    """Load camera intrinsics and distortion from JSON calibration file.

    Returns:
        (fx, fy, cx, cy, distortion_coeffs)
    """
    try:
        with open(calib_path, "r") as f:
            calib = json.load(f)
        cm = calib["camera_matrix"]
        dc = calib.get("distortion_coefficients", [0.0, 0.0, 0.0, 0.0, 0.0])
        fx = cm[0][0]
        fy = cm[1][1]
        cx = cm[0][2]
        cy = cm[1][2]
        logging.info(
            "Camera calibration loaded: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
            fx, fy, cx, cy,
        )
        return fx, fy, cx, cy, dc
    except Exception as e:
        logging.warning(
            "Cannot load calibration (%s), using defaults: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
            e, DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY,
        )
        return DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY, [0.0, 0.0, 0.0, 0.0, 0.0]
