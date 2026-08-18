"""
KP2026 DepthFusion — YOLO bbox + D435i stereo depth sensor fusion.

Combines visual detection with depth measurements for:
  - Fused distance estimation (weighted blend based on sensor reliability)
  - Safe speed computation (speed scales with multi-sensor agreement)
  - Pass-gate decision (both sensors must agree gate is passable)

Aperture detection runs in the DepthSafetyMonitor background thread.
Fusion reads cached aperture result from SafetyState with zero overhead.
"""

import logging
from dataclasses import dataclass
from typing import Optional

import config as cfg
from core.depth_safety import ClearanceResult, SafetyState
from core.guidance import VisualTarget

logger = logging.getLogger(__name__)


@dataclass
class FusedTarget:
    """Target gate with fused YOLO + depth measurements."""

    yolo_cx: float
    yolo_cy: float
    yolo_width: float
    yolo_height: float
    yolo_distance: float
    yolo_confidence: float

    depth_distance: float
    depth_available: bool
    depth_noise: float
    depth_clearance: Optional[ClearanceResult] = None

    fused_distance: float = 99.0
    fusion_confidence: float = 0.0
    sensor_agreement: float = 1.0
    safe_speed_multiplier: float = 1.0
    pass_ready_yolo: bool = False
    pass_ready_depth: bool = False
    pass_ready_fused: bool = False

    # Aperture from depth thread (cached, zero-cost to read)
    aperture_valid: bool = False
    aperture_cx: float = 0.0
    aperture_cy: float = 0.0
    aperture_left: float = 0.0
    aperture_right: float = 0.0
    aperture_top: float = 0.0
    aperture_bottom: float = 0.0
    aperture_w: float = 0.0
    aperture_h: float = 0.0
    aperture_clearance_m: float = 99.0

    warning: str = ""

    def __post_init__(self):
        if self.depth_clearance is not None:
            self.pass_ready_depth = self.depth_clearance.is_clear
        self.pass_ready_yolo = (
            self.yolo_width / cfg.CAMERA_WIDTH >= cfg.PASS_BBOX_RATIO
        )
        self.pass_ready_fused = self.pass_ready_yolo and self.pass_ready_depth


class DepthFusion:
    """Fuses YOLO bbox-based and D435i depth-based measurements.

    Aperture: reads from SafetyState (computed in depth thread, zero main-thread cost).
    """

    def __init__(
        self,
        bbox_reliable_min_m: float = 0.8,
        bbox_reliable_max_m: float = 8.0,
        depth_reliable_min_m: float = 0.3,
        depth_reliable_max_m: float = 5.5,
        fusion_blend_start_m: float = 1.0,
        fusion_blend_end_m: float = 4.0,
        disagreement_threshold_ratio: float = 0.40,
        noise_speed_penalty: float = 0.25,
        camera_width: int = cfg.CAMERA_WIDTH,
    ):
        self.bbox_reliable_min = bbox_reliable_min_m
        self.bbox_reliable_max = bbox_reliable_max_m
        self.depth_reliable_min = depth_reliable_min_m
        self.depth_reliable_max = depth_reliable_max_m
        self.fusion_blend_start = fusion_blend_start_m
        self.fusion_blend_end = fusion_blend_end_m
        self.disagreement_threshold_ratio = disagreement_threshold_ratio
        self.noise_speed_penalty = noise_speed_penalty
        self.camera_width = camera_width

    def _bbox_reliability(self, d: float) -> float:
        if d < self.bbox_reliable_min:
            return 0.3 + 0.7 * d / self.bbox_reliable_min
        if d < self.bbox_reliable_max:
            return 1.0
        return max(0.15, 1.0 - (d - self.bbox_reliable_max) / (self.bbox_reliable_max * 0.5))

    def _depth_reliability(self, d: float, noise: float) -> float:
        if d < self.depth_reliable_min:
            return 0.0
        if d < self.depth_reliable_max:
            rf = 1.0 - (d / self.depth_reliable_max) * 0.6
        else:
            rf = max(0.05, 0.4 - (d - self.depth_reliable_max) * 0.3)
        return rf * max(0.0, 1.0 - noise / 0.3)

    def fuse_distance(self, yolo_d: float, depth_d: float,
                      depth_noise: float = 0.0, depth_avail: bool = True) -> tuple:
        if not depth_avail or depth_d >= 98.0:
            return yolo_d, self._bbox_reliability(yolo_d), 1.0

        br = self._bbox_reliability(yolo_d)
        dr = self._depth_reliability(depth_d, depth_noise)
        diff = abs(yolo_d - depth_d) / max(yolo_d, 0.1)
        agree = 1.0 - min(1.0, diff / self.disagreement_threshold_ratio)

        if yolo_d < self.fusion_blend_start:
            alpha = 0.85
        elif yolo_d < self.fusion_blend_end:
            t = (yolo_d - self.fusion_blend_start) / (self.fusion_blend_end - self.fusion_blend_start)
            alpha = 0.85 - t * 0.70
        else:
            alpha = 0.15
        alpha *= dr / max(br, 0.01)

        fused = yolo_d * (1.0 - alpha) + depth_d * alpha
        return fused, max(br, dr) * agree, agree

    def fuse_target(self, target: VisualTarget, safety_state: Optional[SafetyState],
                    yolo_confidence: float) -> FusedTarget:
        dd, dn, da = 99.0, 0.0, False
        clearance = None
        ap = (False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99.0)

        if safety_state and safety_state.depth_available:
            da = True
            dd = safety_state.measured_distance
            dn = safety_state.noise_level
            clearance = safety_state.gate_clearance
            ap = (safety_state.aperture_valid, safety_state.aperture_cx,
                  safety_state.aperture_cy, safety_state.aperture_left,
                  safety_state.aperture_right, safety_state.aperture_top,
                  safety_state.aperture_bottom, safety_state.aperture_w,
                  safety_state.aperture_h, safety_state.aperture_clearance_m)

        fd, fc, sa = self.fuse_distance(target.est_distance, dd, dn, da)

        ft = FusedTarget(
            yolo_cx=target.cx, yolo_cy=target.cy,
            yolo_width=target.width, yolo_height=target.height,
            yolo_distance=target.est_distance, yolo_confidence=yolo_confidence,
            depth_distance=dd, depth_available=da,
            depth_noise=dn, depth_clearance=clearance,
            fused_distance=fd, fusion_confidence=fc, sensor_agreement=sa,
            aperture_valid=ap[0], aperture_cx=ap[1], aperture_cy=ap[2],
            aperture_left=ap[3], aperture_right=ap[4],
            aperture_top=ap[5], aperture_bottom=ap[6],
            aperture_w=ap[7], aperture_h=ap[8], aperture_clearance_m=ap[9],
        )
        ft.__post_init__()
        ft.safe_speed_multiplier = self._compute_speed_multiplier(ft)
        ft.warning = self._build_warning(ft)
        return ft

    def _compute_speed_multiplier(self, ft: FusedTarget) -> float:
        m = 1.0
        if ft.fusion_confidence < 0.3:
            m = 0.35
        elif ft.fusion_confidence < 0.6:
            m = 0.6
        elif ft.fusion_confidence < 0.8:
            m = 0.8
        if ft.depth_available:
            if ft.depth_clearance and not ft.depth_clearance.is_clear:
                m = min(m, 0.3)
            if ft.sensor_agreement < 0.4:
                m = min(m, 0.4)
        if ft.depth_noise > cfg.DEPTH_NOISE_WARN_THRESHOLD and ft.depth_available:
            m = min(m, 1.0 - self.noise_speed_penalty)
        if ft.yolo_confidence < cfg.CONF_THRESHOLD * 2:
            m = min(m, 0.7)
        if ft.aperture_valid:
            if ft.aperture_clearance_m < 0.3:
                m = min(m, 0.3)
            elif ft.aperture_clearance_m < 0.5:
                m = min(m, 0.6)
        return max(m, 0.15)

    def should_pass_fused(self, ft: FusedTarget) -> tuple:
        if not ft.pass_ready_yolo:
            r = ft.yolo_width / self.camera_width
            return False, f"YOLO: bbox {r*100:.0f}% < {cfg.PASS_BBOX_RATIO*100:.0f}%"
        if ft.depth_available and not ft.pass_ready_depth:
            if ft.depth_clearance:
                return False, f"Depth: {ft.depth_clearance.warning}"
            return False, "Depth: no clearance data"
        if ft.fusion_confidence < 0.4:
            return False, f"Fusion confidence too low ({ft.fusion_confidence:.2f})"
        if ft.sensor_agreement < 0.3:
            return False, f"Sensor disagreement ({ft.sensor_agreement:.2f})"
        if ft.depth_available and ft.depth_noise > cfg.DEPTH_NOISE_WARN_THRESHOLD * 2:
            return False, f"Depth noise too high ({ft.depth_noise:.3f}m)"
        return True, "OK"

    def _build_warning(self, ft: FusedTarget) -> str:
        p = []
        if ft.fusion_confidence < 0.5:
            p.append(f"low_conf({ft.fusion_confidence:.2f})")
        if ft.sensor_agreement < 0.5:
            p.append(f"disagree(bbox={ft.yolo_distance:.1f} dep={ft.depth_distance:.1f})")
        if ft.depth_available and ft.depth_noise > cfg.DEPTH_NOISE_WARN_THRESHOLD:
            p.append(f"noise({ft.depth_noise:.3f})")
        if ft.depth_clearance and ft.depth_clearance.warning:
            p.append(ft.depth_clearance.warning[:40])
        if ft.aperture_valid and ft.aperture_clearance_m < 0.4:
            p.append(f"TIGHT({ft.aperture_clearance_m:.2f}m)")
        return "; ".join(p) if p else ""

    def compute_safe_speed(self, base: float, ft: FusedTarget,
                           min_s: float = 0.3, max_s: float = 6.0) -> float:
        return max(min_s, min(max_s, base * ft.safe_speed_multiplier))
