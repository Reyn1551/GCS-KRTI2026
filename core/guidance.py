"""
KP2026 Visual Guidance — bounding-box based visual servoing with camera tilt compensation.

Converts YOLO detections into velocity commands for the flight controller.
"""

import logging
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from config import (
    CAMERA_TILT_DEG,
    CAMERA_TILT_RAD,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    CRUISE_SPEED_MAX,
    CRUISE_SPEED_MIN,
    GATE_REAL_WIDTH_M,
    KD_X_FAST,
    KD_X_PREC,
    KI_X_FAST,
    KI_X_PREC,
    KP_X_FAST,
    KP_X_PREC,
    PASS_BBOX_RATIO,
    VY_LIMIT,
    VZ_LIMIT,
)
from core.vision_utils import lerp


@dataclass
class GateDetection:
    """Single gate detection from YOLO."""

    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    cx: float = 0.0
    cy: float = 0.0
    width: float = 0.0
    height: float = 0.0

    def __post_init__(self):
        self.cx = (self.x1 + self.x2) / 2.0
        self.cy = (self.y1 + self.y2) / 2.0
        self.width = self.x2 - self.x1
        self.height = self.y2 - self.y1


@dataclass
class VisualTarget:
    """Target gate information — bbox converted to spatial data."""

    cx: float
    cy: float
    width: float
    height: float
    est_distance: float
    confidence: float
    offset_x_px: float = 0.0
    offset_y_px: float = 0.0
    offset_x_m: float = 0.0
    offset_y_m: float = 0.0
    bbox_ratio: float = 0.0

    def __post_init__(self):
        self.bbox_ratio = self.width / CAMERA_WIDTH


@dataclass
class VelocityCommand:
    vx: float = 0.0  # forward (body NED)
    vy: float = 0.0  # right
    vz: float = 0.0  # down


class VisualGuidance:
    """
    Bounding-box based visual guidance with camera tilt compensation.

    Camera tilt 20° UP → target alignment Y offset below frame center.
    During forward flight (body pitch ~15-20°), camera becomes ~horizontal,
    so target Y offset is reduced.
    """

    def __init__(self, fx: float, fy: float, cx: float, cy: float):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.target_x = cx

        # Two alignment modes
        self.target_y_hover = cy + fy * math.tan(CAMERA_TILT_RAD)
        self.target_y_forward = cy + fy * math.tan(math.radians(5.0))
        self._current_target_y = self.target_y_hover

        # PID state
        self._ix = 0.0
        self._ex_prev = 0.0
        self._iy = 0.0
        self._ey_prev = 0.0
        self._smooth_target: Optional[VisualTarget] = None
        self._smooth_cmd = VelocityCommand()
        self._max_integral = 3.0
        self._precision_mode = False

        logging.info(
            "VisualGuidance: target_y hover=%.0fpx forward=%.0fpx target_x=%.0fpx",
            self.target_y_hover, self.target_y_forward, self.target_x,
        )

    def set_precision_mode(self, enabled: bool):
        """Switch between FAST (hover target) and PRECISION (forward target)."""
        self._precision_mode = enabled
        target = self.target_y_forward if enabled else self.target_y_hover
        self._current_target_y = lerp(self._current_target_y, target, 0.3)

    def estimate_distance(self, bbox_width_px: float) -> float:
        """Estimate distance from bbox width (similar triangles)."""
        if bbox_width_px < 1:
            return 99.0
        return max(0.3, min(99.0, (GATE_REAL_WIDTH_M * self.fx) / bbox_width_px))

    def gate_to_target(self, gate: GateDetection) -> VisualTarget:
        """Convert GateDetection → VisualTarget."""
        dist = self.estimate_distance(gate.width)
        target = VisualTarget(
            cx=gate.cx, cy=gate.cy,
            width=gate.width, height=gate.height,
            est_distance=dist, confidence=gate.confidence,
        )
        target.offset_x_px = gate.cx - self.target_x
        target.offset_y_px = gate.cy - self._current_target_y
        if dist > 0.1:
            target.offset_x_m = target.offset_x_px * dist / self.fx
            target.offset_y_m = target.offset_y_px * dist / self.fy
        return target

    def smooth_target(self, target: VisualTarget, alpha: float = 0.5) -> VisualTarget:
        """EMA smoothing of bbox."""
        if self._smooth_target is None:
            self._smooth_target = target
            return target
        st = self._smooth_target
        st.cx = lerp(st.cx, target.cx, alpha)
        st.cy = lerp(st.cy, target.cy, alpha)
        st.width = lerp(st.width, target.width, alpha)
        st.height = lerp(st.height, target.height, alpha)
        st.est_distance = self.estimate_distance(st.width)
        st.confidence = target.confidence
        st.bbox_ratio = st.width / CAMERA_WIDTH
        st.offset_x_px = st.cx - self.target_x
        st.offset_y_px = st.cy - self._current_target_y
        if st.est_distance > 0.1:
            st.offset_x_m = st.offset_x_px * st.est_distance / self.fx
            st.offset_y_m = st.offset_y_px * st.est_distance / self.fy
        return st

    def compute_command(self, target: VisualTarget, dt: float, gentle: bool = False) -> VelocityCommand:
        """Velocity command: forward speed scales with bbox (close=faster), lateral centering. vZ=0."""
        return self._compute_command_impl(target, dt, gentle, None)

    def compute_command_fused(self, target: VisualTarget, dt: float, fused_target=None,
                              gentle: bool = False) -> VelocityCommand:
        """Velocity command using fused YOLO+depth data for speed + centering."""
        return self._compute_command_impl(target, dt, gentle, fused_target)

    def _compute_command_impl(self, target: VisualTarget, dt: float,
                               gentle: bool = False, fused_target=None) -> VelocityCommand:
        if self._precision_mode:
            kp_x, ki_x, kd_x = KP_X_PREC, KI_X_PREC, KD_X_PREC
        else:
            kp_x, ki_x, kd_x = KP_X_FAST, KI_X_FAST, KD_X_FAST

        # ── Fused distance for speed profile ──────────────────────────
        eff_distance = target.est_distance
        if fused_target is not None:
            eff_distance = fused_target.fused_distance

        # ── Forward speed ──────────────────────────────────────────────
        # slow when far (time to center) → fast when close (pass momentum)
        br = target.bbox_ratio
        if br < 0.08:
            speed = CRUISE_SPEED_MIN
        elif br < 0.40:
            t = (br - 0.08) / 0.32
            speed = CRUISE_SPEED_MIN + t * (CRUISE_SPEED_MAX - CRUISE_SPEED_MIN)
        else:
            speed = CRUISE_SPEED_MAX

        # ── Fusion: apply safe speed multiplier ────────────────────────
        if fused_target is not None:
            speed *= fused_target.safe_speed_multiplier
            speed = max(0.3, min(CRUISE_SPEED_MAX, speed))

        # ── Aperture-based centering offsets ───────────────────────────
        offset_x_px = target.offset_x_px
        offset_y_px = target.offset_y_px

        if fused_target is not None and fused_target.aperture_valid:
            # Use depth-detected TRUE aperture center as primary centering target
            # This overrides YOLO bbox center, which may be shifted when
            # YOLO only detects part of the gate (common at close range)
            offset_x_px = fused_target.aperture_cx - self.target_x
            offset_y_px = fused_target.aperture_cy - self._current_target_y

            # Tight clearance → slow down further
            if fused_target.aperture_clearance_m < 0.3:
                speed = min(speed, 0.4)
            elif fused_target.aperture_clearance_m < 0.5:
                speed = min(speed, 0.8)

        ex_norm = offset_x_px / (CAMERA_WIDTH / 2.0)
        ey_norm = offset_y_px / (CAMERA_HEIGHT / 2.0)

        p_x = kp_x * ex_norm

        if abs(ex_norm) > 0.05:
            self._ix += ex_norm * dt
            self._ix = np.clip(self._ix, -self._max_integral, self._max_integral)

        i_x = ki_x * self._ix
        d_x = kd_x * (ex_norm - self._ex_prev) / max(dt, 0.001)
        self._ex_prev = ex_norm
        vy = np.clip(p_x + i_x + d_x, -VY_LIMIT, VY_LIMIT)

        # ── Vertical PID (aperture-aware, active when depth available) ─
        vz = 0.0
        if fused_target is not None and fused_target.aperture_valid and abs(offset_y_px) > 8:
                kp_y = KP_X_PREC * 0.7 if self._precision_mode else KP_X_FAST * 0.6
                ki_y = KI_X_PREC * 0.5 if self._precision_mode else KI_X_FAST * 0.4
                kd_y = KD_X_PREC * 0.5 if self._precision_mode else KD_X_FAST * 0.4

                p_y = kp_y * ey_norm
                self._iy += ey_norm * dt
                self._iy = np.clip(self._iy, -self._max_integral * 0.5, self._max_integral * 0.5)
                i_y = ki_y * self._iy
                d_y = kd_y * (ey_norm - self._ey_prev) / max(dt, 0.001)
                self._ey_prev = ey_norm
                vz = np.clip(p_y + i_y + d_y, -VZ_LIMIT, VZ_LIMIT)

        dist_factor = np.clip(2.0 - eff_distance / 5.0, 0.8, 1.5)
        vy *= dist_factor

        cmd = VelocityCommand(vx=speed, vy=vy, vz=vz)

        self._smooth_cmd.vx = lerp(self._smooth_cmd.vx, cmd.vx, 0.35)
        self._smooth_cmd.vy = lerp(self._smooth_cmd.vy, cmd.vy, 0.45)
        self._smooth_cmd.vz = lerp(self._smooth_cmd.vz, cmd.vz, 0.40)

        return self._smooth_cmd

    def should_pass(self, target: VisualTarget) -> bool:
        """Check if gate is large enough to pass through."""
        return target.bbox_ratio >= PASS_BBOX_RATIO

    def is_aligned(self, target: VisualTarget, threshold_px: float = 25) -> bool:
        """Check if drone is aligned with target point."""
        return (
            abs(target.offset_x_px) < threshold_px
            and abs(target.offset_y_px) < threshold_px * 2
        )

    def reset(self):
        self._ix = 0.0
        self._ex_prev = 0.0
        self._iy = 0.0
        self._ey_prev = 0.0
        self._smooth_target = None
        self._smooth_cmd = VelocityCommand()
        self._current_target_y = self.target_y_hover
