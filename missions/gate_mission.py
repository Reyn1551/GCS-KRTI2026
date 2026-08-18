#!/usr/bin/env python3
"""
KP2026 — GATE MISSION v4.0
ArduPilot MAVLink + Hailo YOLO + RealSense RGB + Visual Servoing

State machine: SEARCH → FAST_APPROACH → PRECISION_ALIGN → PASS → COMPLETE + EMERGENCY

Uses core/ library for all shared components (HailoDetector, SimpleTracker,
VisualGuidance, DroneController) and config.py for all constants.
"""

import argparse
import logging
import os
import signal
import sys
import time
from typing import Dict, List, Optional, Tuple, Union

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
import pyrealsense2 as rs

import config as cfg
from core.camera import load_camera_calibration
from core.depth_safety import (
    ClearanceResult,
    DepthSafetyMonitor,
    SafetyState,
    create_monitor,
)
from core.drone import DroneController, MockDrone
from core.fusion import DepthFusion, FusedTarget
from core.guidance import GateDetection, VelocityCommand, VisualGuidance, VisualTarget
from core.hailo_detector import HailoDetector
from core.tracker import SimpleTracker


# ═══════════════════════════════════════════════════════════════════════════
# Utility: deduplicate overlapping detections
# ═══════════════════════════════════════════════════════════════════════════


def _iou(bbox_a: Tuple, bbox_b: Tuple) -> float:
    xa1, ya1, xa2, ya2 = bbox_a
    xb1, yb1, xb2, yb2 = bbox_b
    xi1 = max(xa1, xb1)
    yi1 = max(ya1, yb1)
    xi2 = min(xa2, xb2)
    yi2 = min(ya2, yb2)
    inter = max(0, xi2 - xi1) * max(0, yi2 - yi1)
    area_a = (xa2 - xa1) * (ya2 - ya1)
    area_b = (xb2 - xb1) * (yb2 - yb1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def _dedup_overlapping(detections: List[Dict], iou_thres: float = 0.35) -> List[Dict]:
    """Remove overlapping detections of same class, keep highest confidence."""
    if len(detections) <= 1:
        return detections

    # Sort by confidence descending
    sorted_dets = sorted(detections, key=lambda d: d["score"], reverse=True)
    keep = []

    for det in sorted_dets:
        suppressed = False
        for kept in keep:
            if det["class_id"] == kept["class_id"]:
                iou_val = _iou(det["bbox"], kept["bbox"])
                if iou_val > iou_thres:
                    suppressed = True
                    break
        if not suppressed:
            keep.append(det)

    return keep


# ═══════════════════════════════════════════════════════════════════════════
# GATE MISSION STATE MACHINE
# ═══════════════════════════════════════════════════════════════════════════


class GateMission:
    """State machine for gate mission with visual servoing."""

    STATE_SEARCH = "SEARCH"
    STATE_CENTER = "CENTER"
    STATE_FAST_APPROACH = "FAST_APPROACH"
    STATE_PRECISION_ALIGN = "PRECISION_ALIGN"
    STATE_PASS = "PASS"
    STATE_COMPLETE = "COMPLETE"
    STATE_EMERGENCY = "EMERGENCY"

    def __init__(
        self,
        takeoff_alt: float = 3.0,
        gate_count: int = 1,
        vision_only: bool = False,
        dry_run: bool = False,
        conf_threshold: float = cfg.CONF_THRESHOLD,
        gate_width_m: float = cfg.GATE_REAL_WIDTH_M,
        gate_height_m: float = cfg.GATE_REAL_HEIGHT_M,
        camera_tilt_deg: float = cfg.CAMERA_TILT_DEG,
        hud_mode: str = "minimal",
        record: bool = False,
        speed_min: float = cfg.CRUISE_SPEED_MIN,
        speed_max: float = cfg.CRUISE_SPEED_MAX,
        pass_speed: float = cfg.PASS_SPEED,
        depth_enabled: bool = True,
        replay_video: Optional[str] = None,
    ):
        os.makedirs(cfg.LOG_DIR, exist_ok=True)

        log_handlers = [
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(os.path.join(cfg.LOG_DIR, "gate_mission.log")),
        ]
        logging.basicConfig(
            level=logging.INFO,
            format="[%(asctime)s] %(levelname)s: %(message)s",
            handlers=log_handlers,
        )

        # Runtime overrides
        cfg.GATE_REAL_WIDTH_M = gate_width_m
        cfg.GATE_REAL_HEIGHT_M = gate_height_m
        cfg.CAMERA_TILT_DEG = camera_tilt_deg
        cfg.CAMERA_TILT_RAD = __import__("math").radians(camera_tilt_deg)
        cfg.CONF_THRESHOLD = conf_threshold
        cfg.CRUISE_SPEED_MIN = speed_min
        cfg.CRUISE_SPEED_MAX = speed_max
        cfg.PASS_SPEED = pass_speed

        self.takeoff_altitude = takeoff_alt
        self.gate_count_target = gate_count
        self.gates_passed = 0
        self.vision_only = vision_only
        self.dry_run = dry_run
        self.hud_mode = hud_mode
        self.record = record
        self.depth_enabled = depth_enabled
        self.replay_video = replay_video
        self._replay_cap: Optional[cv2.VideoCapture] = None
        self._replay_image: Optional[np.ndarray] = None
        self._replay_paused = False

        self.state = self.STATE_SEARCH
        self.running = True
        self.mission_start_time = 0.0

        # Components
        self.detector: Optional[HailoDetector] = None
        self.tracker: Optional[SimpleTracker] = None
        self.pipeline: Optional[rs.pipeline] = None
        self.guidance: Optional[VisualGuidance] = None
        self.drone: Optional[Union[DroneController, MockDrone]] = None
        self.depth_monitor: Optional[DepthSafetyMonitor] = None
        self._depth_align: Optional[rs.align] = None
        self.fusion: Optional[DepthFusion] = None
        self._depth_scale_x: float = cfg.DEPTH_WIDTH / cfg.CAMERA_WIDTH
        self._depth_scale_y: float = cfg.DEPTH_HEIGHT / cfg.CAMERA_HEIGHT

        # State variables
        self._last_gate_time = 0.0
        self._search_start = 0.0
        self._pass_start = 0.0
        self._approach_frame_count = 0
        self._max_bbox_width = 0.0
        self._pass_gate_lost = False
        self._post_confirmed_time = 0.0
        self._frame_count = 0
        self._centered_frames = 0
        self._best_gate: Optional[GateDetection] = None
        self._last_cmd = VelocityCommand()

        # FPS tracking
        self._fps = 0.0
        self._prev_loop_time = time.perf_counter()
        self._infer_time_sum = 0.0
        self._infer_time_count = 0

        # Depth safety state cache
        self._depth_state: Optional[SafetyState] = None
        self._depth_frame: Optional[np.ndarray] = None
        self._fused_target: Optional[FusedTarget] = None
        self._depth_blocked_frames = 0
        self._last_depth_log = 0.0

        self._video_writer = None

        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    def _on_signal(self, sig, frame):
        logging.warning("Signal %s — emergency shutdown triggered!", sig)
        self.running = False
        self.state = self.STATE_EMERGENCY

    # ── Initialization ────────────────────────────────────────────────────────

    def init_systems(self):
        logging.info("=" * 60)
        logging.info("INITIALIZING GATE MISSION v4.0")
        logging.info("=" * 60)

        # Camera calibration
        kwargs = load_camera_calibration(cfg.CALIB_PATH)
        fx, fy, cx, cy = kwargs[0], kwargs[1], kwargs[2], kwargs[3]

        # HailoDetector
        try:
            self.detector = HailoDetector(cfg.HEF_PATH, conf_thres=cfg.CONF_THRESHOLD)
            logging.info("  [✓] HailoDetector loaded")
        except Exception as e:
            logging.error("  [✗] HailoDetector: %s", e)
            raise

        # Tracker
        self.tracker = SimpleTracker(
            max_miss=cfg.TRACKER_MAX_MISS,
            iou_thres=cfg.TRACKER_IOU_THRES,
            smooth_alpha=cfg.TRACKER_SMOOTH_ALPHA,
        )
        logging.info("  [✓] SimpleTracker initialized")

        # VisualGuidance
        self.guidance = VisualGuidance(fx, fy, cx, cy)
        logging.info("  [✓] VisualGuidance (tilt=%.0f°)", cfg.CAMERA_TILT_DEG)

        # DepthFusion (YOLO + stereo depth sensor fusion)
        if self.depth_enabled:
            self.fusion = DepthFusion()
            logging.info("  [✓] DepthFusion — YOLO+Depth sensor fusion active")
        else:
            self.fusion = None
            logging.info("  [—] DepthFusion SKIPPED (no depth)")

        # RealSense (single pipeline: RGB + Depth) — only if NOT replaying
        if self.replay_video:
            ext = os.path.splitext(self.replay_video)[1].lower()
            if ext in (".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".tif"):
                # Still image — load once, loop same frame
                img = cv2.imread(self.replay_video)
                if img is None:
                    raise FileNotFoundError(f"Cannot open image: {self.replay_video}")
                h, w = img.shape[:2]
                if w != cfg.CAMERA_WIDTH or h != cfg.CAMERA_HEIGHT:
                    img = cv2.resize(img, (cfg.CAMERA_WIDTH, cfg.CAMERA_HEIGHT))
                self._replay_image = img
                self._replay_cap = None
                logging.info("  [✓] Replay: %s (%dx%d image, looping)", self.replay_video, w, h)
            else:
                self._replay_cap = cv2.VideoCapture(self.replay_video)
                if not self._replay_cap.isOpened():
                    raise FileNotFoundError(f"Cannot open video: {self.replay_video}")
                fps_video = self._replay_cap.get(cv2.CAP_PROP_FPS)
                if fps_video > 0:
                    cfg.CAMERA_FPS = int(fps_video)
                self._replay_image = None
                logging.info("  [✓] Replay: %s @ %.1ffps", self.replay_video, fps_video)
            self.pipeline = None
            self._depth_align = None
            self.depth_enabled = False  # No depth from video file
        else:
            try:
                self.pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(
                    rs.stream.color,
                    cfg.CAMERA_WIDTH, cfg.CAMERA_HEIGHT,
                    rs.format.bgr8, cfg.CAMERA_FPS,
                )
                if self.depth_enabled:
                    config.enable_stream(
                        rs.stream.depth,
                        cfg.DEPTH_WIDTH, cfg.DEPTH_HEIGHT,
                        rs.format.z16, cfg.DEPTH_FPS,
                    )
                self.pipeline.start(config)
                self._depth_align = rs.align(rs.stream.color) if self.depth_enabled else None
                for _ in range(10):
                    self.pipeline.wait_for_frames()
                depth_note = " + Depth" if self.depth_enabled else ""
                logging.info("  [✓] RealSense RGB%s @ %dfps", depth_note, cfg.CAMERA_FPS)
            except Exception as e:
                logging.error("  [✗] RealSense: %s", e)
                raise

        # Depth Safety Monitor (D435i stereo depth — anti-collision)
        if self.depth_enabled:
            try:
                self.depth_monitor = create_monitor(
                    enabled=True,
                    depth_width=cfg.DEPTH_WIDTH,
                    depth_height=cfg.DEPTH_HEIGHT,
                    depth_fps=cfg.DEPTH_FPS,
                    external=True,
                )
                if self.depth_monitor:
                    logging.info("  [✓] DepthSafetyMonitor — stereo depth anti-collision active")
                else:
                    logging.warning("  [!] DepthSafetyMonitor — depth unavailable, relying on vision only")
            except Exception as e:
                logging.warning("  [!] DepthSafetyMonitor: %s — continuing without depth", e)
                self.depth_monitor = None
        else:
            logging.info("  [—] DepthSafetyMonitor DISABLED (--no-depth)")
            self.depth_monitor = None

        if self.dry_run:
            logging.info("  [◎] DRY-RUN: drone commands printed, depth safety still active")

        # Drone
        if self.dry_run:
            self.drone = MockDrone(takeoff_alt=self.takeoff_altitude)
            logging.info("  [◎] MockDrone (DRY-RUN)")
        elif not self.vision_only:
            try:
                self.drone = DroneController(port=cfg.SERIAL_PORT, baud=cfg.SERIAL_BAUD)
                logging.info("  [✓] DroneController connected")
            except Exception as e:
                logging.error("  [✗] DroneController: %s", e)
                logging.warning("  → Falling back to vision-only mode")
                self.drone = None
        else:
            logging.info("  [—] Drone SKIPPED (vision-only)")
            self.drone = None

        # Video recording
        if self.record:
            try:
                ts = time.strftime("%Y%m%d_%H%M%S")
                path = os.path.join(cfg.LOG_DIR, f"mission_{ts}.avi")
                fourcc = cv2.VideoWriter_fourcc(*"XVID")
                self._video_writer = cv2.VideoWriter(
                    path, fourcc, 15.0, (cfg.CAMERA_WIDTH, cfg.CAMERA_HEIGHT)
                )
                logging.info("  [✓] Recording → %s", path)
            except Exception as e:
                logging.warning("  [!] Video recording: %s", e)

        logging.info("All systems ready.\n")

    # ── Main Loop ─────────────────────────────────────────────────────────────

    def run(self):
        try:
            self.init_systems()
            self.mission_start_time = time.perf_counter()

            if self.drone:
                logging.info("=== TAKEOFF ===")
                self.drone.takeoff(self.takeoff_altitude)

            self.state = self.STATE_SEARCH
            self._search_start = time.perf_counter()
            logging.info(
                "=== SEARCH (alt=%.1fm, gates=%d) ===",
                self.takeoff_altitude, self.gate_count_target,
            )

            _perf_counter = time.perf_counter
            _max_mission = cfg.MAX_MISSION_TIME
            _min_conf = cfg.MIN_CONFIDENCE_GATE
            _gate_cls = cfg.GATE_CLASS_ID

            while self.running:
                if self._frame_count % 30 == 0:
                    if _perf_counter() - self.mission_start_time > _max_mission:
                        logging.warning("Mission timeout!")
                        self.state = self.STATE_EMERGENCY

                if self._replay_image is not None:
                    frame = self._replay_image.copy()
                    self._depth_frame = None
                elif self._replay_cap:
                    ret, frame = self._replay_cap.read()
                    if not ret:
                        logging.info("Replay: end of video — rewinding")
                        self._replay_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    self._depth_frame = None
                else:
                    frames = self.pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()
                    if not color_frame:
                        continue
                    frame = np.asanyarray(color_frame.get_data())

                # ── Raw depth (native 424x240, NO alignment → zero overhead) ─
                t0 = _perf_counter()

                if not self._replay_image and not self._replay_cap:
                    if self.depth_monitor and self.depth_enabled:
                        depth_frame_raw = frames.get_depth_frame()
                        if depth_frame_raw:
                            self._depth_frame = np.asanyarray(depth_frame_raw.get_data())
                            self.depth_monitor.push_frame(self._depth_frame, t0)
                        else:
                            self._depth_frame = None
                    else:
                        self._depth_frame = None

                dt = t0 - self._prev_loop_time
                self._prev_loop_time = t0
                if dt > 0:
                    self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)
                self._frame_count += 1

                detections = self.detector.infer(frame)
                detections = _dedup_overlapping(detections, iou_thres=0.35)

                infer_ms = (_perf_counter() - t0) * 1000.0
                self._infer_time_sum += infer_ms
                self._infer_time_count += 1

                tracks = self.tracker.update(detections)

                best_gate_track = None
                best_gate_score = 0.0
                for _tid, bbox, cls_id, score, _age in tracks:
                    if cls_id == _gate_cls and score > best_gate_score:
                        best_gate_score = score
                        best_gate_track = (bbox, score)

                if best_gate_track and best_gate_score >= _min_conf:
                    bbox, score = best_gate_track
                    x1, y1, x2, y2 = bbox
                    self._best_gate = GateDetection(
                        x1=x1, y1=y1, x2=x2, y2=y2, confidence=score,
                    )
                    if self.depth_monitor:
                        sx, sy = self._depth_scale_x, self._depth_scale_y
                        self.depth_monitor.set_gate_bbox(
                            int(x1 * sx), int(y1 * sy),
                            int(x2 * sx), int(y2 * sy),
                        )
                else:
                    self._best_gate = None
                    if self.depth_monitor:
                        self.depth_monitor.clear_gate_bbox()

                if self.depth_monitor:
                    self._depth_state = self.depth_monitor.get_state()

                # Compute FusedTarget once per frame (cached for state handlers + HUD)
                if self._best_gate and self.fusion and self._depth_state:
                    target_tmp = self.guidance.gate_to_target(self._best_gate)
                    self._fused_target = self.fusion.fuse_target(
                        target_tmp, self._depth_state, self._best_gate.confidence,
                    )
                else:
                    self._fused_target = None

                state = self.state
                if state == self.STATE_SEARCH:
                    self._handle_search(dt)
                elif state == self.STATE_CENTER:
                    self._handle_center(dt)
                elif state == self.STATE_FAST_APPROACH:
                    self._handle_fast_approach(dt)
                elif state == self.STATE_PRECISION_ALIGN:
                    self._handle_precision_align(dt)
                elif state == self.STATE_PASS:
                    self._handle_pass()
                elif state == self.STATE_COMPLETE:
                    self._handle_complete()
                    break
                elif state == self.STATE_EMERGENCY:
                    self._handle_emergency()
                    break

                if self.hud_mode != "none":
                    self._draw_hud(frame, tracks, detections)
                if self._video_writer:
                    self._video_writer.write(frame)
                cv2.imshow("KP2026 Gate Mission v4", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    self.running = False
                elif key == ord("e"):
                    logging.critical("EMERGENCY!")
                    self.state = self.STATE_EMERGENCY
                elif key == ord("l"):
                    if self.drone:
                        self.drone.land()
                    self.running = False
                elif key == ord("s"):
                    ts = time.strftime("%Y%m%d_%H%M%S")
                    path = os.path.join(cfg.LOG_DIR, f"screenshot_{ts}.png")
                    cv2.imwrite(path, frame)
                    logging.info("Screenshot saved: %s", path)
                elif key == ord("p") and (self._replay_cap or self._replay_image is not None):
                    self._replay_paused = not self._replay_paused
                    logging.info("Replay: %s", "PAUSED" if self._replay_paused else "PLAYING")
                    while self._replay_paused and self.running:
                        k2 = cv2.waitKey(100) & 0xFF
                        if k2 == ord("p"):
                            self._replay_paused = False
                        elif k2 == ord("q"):
                            self.running = False
                        elif k2 == ord(".") and self._replay_cap:
                            ret, frame = self._replay_cap.read()
                            if not ret:
                                break

        except Exception as e:
            logging.exception("Fatal error: %s", e)
            self.state = self.STATE_EMERGENCY
        finally:
            self._cleanup()

    # ── State Handlers ────────────────────────────────────────────────────────

    def _handle_search(self, dt: float):
        gate = self._best_gate
        if gate is not None and gate.confidence >= cfg.CONF_THRESHOLD:
            self.guidance.reset()
            self._last_gate_time = time.perf_counter()
            self._approach_frame_count = 0
            self._max_bbox_width = 0.0
            self._pass_gate_lost = False
            self._centered_frames = 0
            target = self.guidance.gate_to_target(gate)
            logging.info(
                "GATE FOUND! dist~%.1fm, bbox=%.0f%%, conf=%.2f → CENTERING",
                target.est_distance, target.bbox_ratio * 100, gate.confidence,
            )
            self.state = self.STATE_CENTER
            return

        elapsed = time.perf_counter() - self._search_start
        if self.drone:
            # Search pattern simetris: 3.5s kanan, 3.5s kiri (net yaw = 0)
            if (elapsed % 7.0) < 3.5:
                self.drone.send_yaw_rate(0.6)
            else:
                self.drone.send_yaw_rate(-0.6)

    def _handle_center(self, dt: float):
        gate = self._best_gate

        if gate is None:
            if time.perf_counter() - self._last_gate_time > cfg.LOST_GATE_TIMEOUT:
                logging.warning("Gate lost in CENTER → back to SEARCH")
                self.state = self.STATE_SEARCH
                self._search_start = time.perf_counter()
                if self.drone:
                    self.drone.send_velocity_body(0.0, 0.0, 0.0)
                return
            if self.drone:
                self.drone.send_velocity_body(0.0, 0.0, 0.0)
            return

        self._last_gate_time = time.perf_counter()
        target = self.guidance.gate_to_target(gate)
        target = self.guidance.smooth_target(target, 0.50)  # Alpha 0.50 — responsif

        cmd = self.guidance.compute_command(target, dt, gentle=True)
        self._last_cmd = cmd

        aligned = abs(target.offset_x_px) < 30
        if aligned:
            self._centered_frames += 1
        else:
            self._centered_frames = 0

        if self.drone:
            self.drone.send_velocity_body(0.0, cmd.vy, 0.0)

        if self._centered_frames >= 20:
            logging.info(
                "GATE CENTERED! (%d frames, offs=%.0fpx) → FORWARD CRUISE",
                self._centered_frames, target.offset_x_px,
            )
            self._approach_frame_count = 0
            self.guidance.reset()  # Reset PID agar tidak ada integral bawaan
            self.state = self.STATE_FAST_APPROACH
            return

        if self._frame_count % 15 == 0:
            logging.info(
                "  CENTER: offs=(%.0f,%.0f)px vy=%.2f aligned=%d/%d",
                target.offset_x_px, target.offset_y_px,
                cmd.vy, self._centered_frames, 20,
            )

    def _build_fused_target(self, target: VisualTarget, gate: GateDetection) -> Optional[FusedTarget]:
        if self.fusion and self._depth_state:
            return self.fusion.fuse_target(target, self._depth_state, gate.confidence)
        return None

    def _handle_fast_approach(self, dt: float):
        gate = self._best_gate
        self._approach_frame_count += 1

        if gate is None:
            if time.perf_counter() - self._last_gate_time > cfg.LOST_GATE_TIMEOUT:
                logging.warning("Gate lost in FAST_APPROACH → back to SEARCH")
                self.state = self.STATE_SEARCH
                self._search_start = time.perf_counter()
                if self.drone:
                    self.drone.hover()
                return
            if self.drone:
                self.drone.hover()
            return

        self._last_gate_time = time.perf_counter()
        target = self.guidance.gate_to_target(gate)
        target = self.guidance.smooth_target(target, 0.55)

        if target.width > self._max_bbox_width:
            self._max_bbox_width = target.width

        if target.bbox_ratio >= cfg.FAST_TO_PRECISION_RATIO:
            logging.info(
                "→ PRECISION_ALIGN (bbox=%.0f%%, dist~%.1fm)",
                target.bbox_ratio * 100, target.est_distance,
            )
            self.guidance.set_precision_mode(True)
            self._approach_frame_count = 0
            self.state = self.STATE_PRECISION_ALIGN
            return

        ft = self._fused_target
        cmd = self.guidance.compute_command_fused(target, dt, fused_target=ft)
        self._last_cmd = cmd
        if self.drone:
            self.drone.send_velocity_body(cmd.vx, cmd.vy, cmd.vz)

        if self._frame_count % 30 == 0:
            fusion_info = ""
            if ft:
                fusion_info = (
                    f" fused_dist={ft.fused_distance:.1f}m"
                    f" fuse_conf={ft.fusion_confidence:.2f}"
                    f" speed_x={ft.safe_speed_multiplier:.2f}"
                )
            logging.info(
                "  FAST: dist~%.1fm offs=(%.0f,%.0f)px cmd=(vx=%.2f, vy=%.2f, vz=0)%s",
                target.est_distance, target.offset_x_px, target.offset_y_px,
                cmd.vx, cmd.vy, fusion_info,
            )

    def _handle_precision_align(self, dt: float):
        gate = self._best_gate
        self._approach_frame_count += 1

        if gate is None:
            if time.perf_counter() - self._last_gate_time > cfg.LOST_GATE_TIMEOUT:
                logging.warning("Gate lost in PRECISION → back to SEARCH")
                self.state = self.STATE_SEARCH
                self._search_start = time.perf_counter()
                if self.drone:
                    self.drone.hover()
                return
            if self.drone:
                self.drone.hover()
            return

        self._last_gate_time = time.perf_counter()
        target = self.guidance.gate_to_target(gate)
        target = self.guidance.smooth_target(target, 0.45)

        if target.width > self._max_bbox_width:
            self._max_bbox_width = target.width

        ft = self._fused_target

        bbox_big = self.guidance.should_pass(target)
        enough_frames = self._approach_frame_count >= cfg.MIN_APPROACH_FRAMES
        aligned = self.guidance.is_aligned(target, threshold_px=20)

        depth_safe = True
        depth_warning = ""
        fusion_can_pass = True
        fusion_reason = ""

        if ft and ft.depth_available:
            if ft.depth_clearance:
                depth_safe = ft.depth_clearance.is_clear
                depth_warning = ft.depth_clearance.warning

            if not self._depth_state.forward_clear:
                fd = self._depth_state.forward_depth
                if fd < cfg.DEPTH_FORWARD_EMERGENCY_M:
                    logging.critical(
                        "DEPTH EMERGENCY: forward obstacle at %.2fm — ABORT!", fd,
                    )
                    self.state = self.STATE_EMERGENCY
                    return
                depth_safe = False
                depth_warning = f"forward_path blocked: {fd:.1f}m"
                self._depth_blocked_frames += 1
            else:
                self._depth_blocked_frames = 0

            fusion_can_pass, fusion_reason = self.fusion.should_pass_fused(ft)

        can_pass = bbox_big and enough_frames and aligned and depth_safe and fusion_can_pass
        if can_pass:
            fusion_info = ""
            if ft:
                fusion_info = (
                    f" fused_dist={ft.fused_distance:.1f}m"
                    f" fuse_conf={ft.fusion_confidence:.2f}"
                )
            logging.info(
                "PASSING! bbox=%.0f%%, frames=%d, offs=(%.0f,%.0f)px depth_safe=True%s",
                target.bbox_ratio * 100, self._approach_frame_count,
                target.offset_x_px, target.offset_y_px, fusion_info,
            )
            self.state = self.STATE_PASS
            self._pass_start = time.perf_counter()
            return

        if self._frame_count % 30 == 0 and bbox_big:
            reasons = []
            if not enough_frames:
                reasons.append(f"wait_frames({self._approach_frame_count}/{cfg.MIN_APPROACH_FRAMES})")
            if not aligned:
                reasons.append(f"align(offs={target.offset_x_px:.0f},{target.offset_y_px:.0f})")
            if not depth_safe:
                reasons.append(f"depth({depth_warning})")
            if not fusion_can_pass:
                reasons.append(f"fusion({fusion_reason})")
            if reasons:
                logging.info("  HOLD: %s", ", ".join(reasons))

        cmd = self.guidance.compute_command_fused(target, dt, fused_target=ft)
        self._last_cmd = cmd
        vy_final = cmd.vy
        if target.bbox_ratio >= cfg.FORCE_STRAIGHT_RATIO:
            vy_final = 0.0

        vx_final = cmd.vx
        if not depth_safe and self._depth_blocked_frames > 5:
            vx_final = min(vx_final, 0.5)
            if self._frame_count % 30 == 0:
                logging.info("  DEPTH HOLD: reducing vx to %.2f due to depth blocker", vx_final)

        if self.drone:
            self.drone.send_velocity_body(vx_final, vy_final, cmd.vz)

        if self._frame_count % 30 == 0:
            label = ""
            if vy_final == 0.0 and cmd.vy != 0.0:
                label = " (forced straight)"
            if vx_final != cmd.vx:
                label += f" (depth-limited vx)"
            fusion_label = ""
            if ft:
                fusion_label = (
                    f" fusion(d={ft.fused_distance:.1f}m conf={ft.fusion_confidence:.2f}"
                    f" mult={ft.safe_speed_multiplier:.2f})"
                )
            logging.info(
                "  PREC: dist~%.1fm offs=(%.0f,%.0f)px cmd=(vx=%.2f, vy=%.2f, vz=0)%s%s",
                target.est_distance, target.offset_x_px, target.offset_y_px,
                vx_final, vy_final, label, fusion_label,
            )

    def _handle_pass(self):
        gate = self._best_gate
        elapsed = time.perf_counter() - self._pass_start

        current_w = gate.width if gate is not None else 0
        if gate is not None and gate.width > self._max_bbox_width:
            self._max_bbox_width = gate.width

        pass_threshold_px = cfg.CAMERA_WIDTH * cfg.PASS_BBOX_RATIO

        # ── Fusion-based safe speed during pass ────────────────────────
        safe_pass_speed = cfg.PASS_SPEED
        if gate is not None:
            target = self.guidance.gate_to_target(gate)
            ft = self._fused_target
            if ft:
                safe_pass_speed = self.fusion.compute_safe_speed(
                    cfg.PASS_SPEED, ft, min_speed=1.0, max_speed=cfg.PASS_SPEED
                )

        # ── Depth emergency during pass ────────────────────────────────
        if self.depth_monitor and self._depth_state and self._depth_state.depth_available:
            if not self._depth_state.forward_clear:
                fd = self._depth_state.forward_depth
                if fd < cfg.DEPTH_FORWARD_EMERGENCY_M:
                    logging.critical(
                        "DEPTH EMERGENCY during PASS: obstacle at %.2fm — ABORT!", fd,
                    )
                    self.state = self.STATE_EMERGENCY
                    return
                safe_pass_speed = min(safe_pass_speed, 0.8)

        # Condition 1: Gate was large → now disappeared
        if self._max_bbox_width >= pass_threshold_px and gate is None and elapsed > 0.3:
            self._pass_gate_lost = True
            self._post_confirmed_time = time.perf_counter()
            logging.info(
                "PASS CONFIRMED: gate was %.0fpx (≥%.0f%%) → DISAPPEARED!",
                self._max_bbox_width, cfg.PASS_BBOX_RATIO * 100,
            )

        # Condition 2: Gate shrunk drastically
        if (
            self._max_bbox_width >= pass_threshold_px
            and gate is not None
            and current_w < self._max_bbox_width * 0.35
        ):
            self._pass_gate_lost = True
            self._post_confirmed_time = time.perf_counter()
            logging.info(
                "PASS CONFIRMED: gate shrunk %.0f→%.0fpx!",
                self._max_bbox_width, current_w,
            )

        # Condition 3: Timeout
        if elapsed > 4.0:
            self._pass_gate_lost = True
            self._post_confirmed_time = time.perf_counter()
            logging.info("PASS: Timeout — assuming passed.")

        # Transition
        if self._pass_gate_lost:
            self.gates_passed += 1
            logging.info(
                "=== GATE %d/%d PASSED! (elapsed=%.1fs) ===",
                self.gates_passed, self.gate_count_target, elapsed,
            )
            post_elapsed = time.perf_counter() - self._post_confirmed_time
            if self.drone:
                if post_elapsed < cfg.PASS_POST_SECONDS:
                    self.drone.send_velocity_body(safe_pass_speed, 0.0, 0.0)
                else:
                    self.drone.hover()
                    time.sleep(0.3)
                    self.drone.land()
                    if self.gates_passed >= self.gate_count_target:
                        self.state = self.STATE_COMPLETE
                    else:
                        self.state = self.STATE_SEARCH
                        self._search_start = time.perf_counter()
                    return
                if self._frame_count % 15 == 0:
                    logging.info("  POST-PASS: %.1fs / %.1fs", post_elapsed, cfg.PASS_POST_SECONDS)
                return
            else:
                if self.gates_passed >= self.gate_count_target:
                    self.state = self.STATE_COMPLETE
                else:
                    self.state = self.STATE_SEARCH
                    self._search_start = time.perf_counter()
                return

        if self.drone:
            self.drone.send_velocity_body(safe_pass_speed, 0.0, 0.0)

        if self._frame_count % 15 == 0:
            pct = (current_w / cfg.CAMERA_WIDTH) * 100 if current_w > 0 else 0
            logging.info(
                "  PASS [%.1fs]: gate=%.0f%% (max=%.0f%%)",
                elapsed, pct, self._max_bbox_width / cfg.CAMERA_WIDTH * 100,
            )

    def _handle_complete(self):
        elapsed = time.perf_counter() - self.mission_start_time
        logging.info("=" * 60)
        logging.info("MISSION COMPLETE: %d gates in %.0fs", self.gates_passed, elapsed)
        logging.info("=" * 60)
        self.running = False

    def _handle_emergency(self):
        logging.critical("EMERGENCY — initiating immediate landing!")
        if self.drone:
            try:
                for _ in range(5):
                    self.drone.hover()
                    time.sleep(0.1)
                self.drone.rtl()
            except Exception:
                pass
        self.running = False

    # ── HUD ───────────────────────────────────────────────────────────────────

    def _draw_hud(self, frame: np.ndarray, tracks: List, raw_detections: List[Dict]):
        h, w = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Center crosshair
        cv2.drawMarker(frame, (w // 2, h // 2), (100, 100, 100), cv2.MARKER_CROSS, 20, 1)

        # Target alignment point
        if self.guidance:
            target_y = int(self.guidance._current_target_y)
            target_x = int(self.guidance.target_x)
            cv2.line(frame, (0, target_y), (w, target_y), (0, 140, 255), 1)
            cv2.drawMarker(frame, (target_x, target_y), (0, 200, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.circle(frame, (target_x, target_y), 6, (0, 200, 255), 1)

        # Draw best gate only (no overlapping boxes)
        if self._best_gate is not None:
            g = self._best_gate
            x1, y1, x2, y2 = int(g.x1), int(g.y1), int(g.x2), int(g.y2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cx_g = (x1 + x2) // 2
            cy_g = (y1 + y2) // 2
            cv2.circle(frame, (cx_g, cy_g), 4, (0, 255, 255), -1)

            # ── Aperture overlay — depth-detected opening edges ─────────
            if self._fused_target and self._fused_target.aperture_valid:
                ft = self._fused_target
                ax1 = int(ft.aperture_left)
                ay1 = int(ft.aperture_top)
                ax2 = int(ft.aperture_right)
                ay2 = int(ft.aperture_bottom)
                cv2.rectangle(frame, (ax1, ay1), (ax2, ay2), (0, 255, 128), 2)
                acx = int(ft.aperture_cx)
                acy = int(ft.aperture_cy)
                cv2.drawMarker(frame, (acx, acy), (0, 255, 128), cv2.MARKER_CROSS, 14, 2)
                cv2.putText(
                    frame,
                    f"OPEN {ft.aperture_w:.0f}x{ft.aperture_h:.0f}px clr={ft.aperture_clearance_m:.2f}m",
                    (ax1, max(ay1 - 5, 10)), font, 0.3, (0, 255, 128), 1,
                )
                if self.guidance.target_x:
                    cv2.line(
                        frame,
                        (int(self.guidance.target_x), int(self.guidance._current_target_y)),
                        (acx, acy), (0, 255, 128), 1,
                    )

        if self.hud_mode == "minimal":
            avg_infer = (
                self._infer_time_sum / self._infer_time_count
                if self._infer_time_count > 0 else 0.0
            )

            depth_text = ""
            depth_color = (0, 255, 0)
            if self._depth_state and self._depth_state.depth_available:
                fd = self._depth_state.forward_depth
                depth_text = f" D:{fd:.1f}m"
                if not self._depth_state.forward_clear:
                    depth_color = (0, 0, 255)
                elif self._depth_state.noise_level > cfg.DEPTH_NOISE_WARN_THRESHOLD:
                    depth_color = (0, 200, 255)
                gc = self._depth_state.gate_clearance
                if gc and gc.warning:
                    depth_text += f" {gc.warning[:20]}"

            fusion_text = ""
            if self._fused_target:
                ft = self._fused_target
                fusion_text = f" F:{ft.fused_distance:.1f}m C:{ft.fusion_confidence:.2f}"

            cv2.putText(
                frame,
                f"FPS:{self._fps:.0f} INF:{avg_infer:.0f}ms{depth_text}{fusion_text}",
                (8, 22), font, 0.45, depth_color, 1,
            )

            if self._depth_state and self._depth_state.depth_available:
                colormap = self.depth_monitor.get_depth_colormap()
                if colormap is not None and colormap.shape[0] > 0:
                    h_cm, w_cm = colormap.shape[:2]
                    scale_cm = 0.25
                    cm_small = cv2.resize(
                        colormap,
                        (int(w_cm * scale_cm), int(h_cm * scale_cm)),
                        interpolation=cv2.INTER_NEAREST,
                    )
                    cmh, cmw = cm_small.shape[:2]
                    cm_x = w - cmw - 5
                    cm_y = h - cmh - 5
                    if cm_y > 0 and cm_x > 0:
                        frame[cm_y:cm_y + cmh, cm_x:cm_x + cmw] = cm_small
                        cv2.putText(
                            frame, "DEPTH", (cm_x, cm_y - 4),
                            font, 0.35, depth_color, 1,
                        )

            return

        # Full HUD (tracks with labels, non-gate detections, arrow, panel)
        for _tid, bbox, cls_id, score, _age in tracks:
            if cls_id != cfg.GATE_CLASS_ID:
                continue
            x1, y1, _x2, y2 = bbox
            cv2.putText(
                frame, f"GATE {score:.2f}", (x1, max(y1 - 8, 10)),
                font, 0.4, (0, 255, 0), 2,
            )

        for det in raw_detections:
            cls_id = det["class_id"]
            if cls_id == cfg.GATE_CLASS_ID:
                continue
            x1, y1, x2, y2 = det["bbox"]
            score = det["score"]
            label = cfg.LABELS[cls_id] if cls_id < len(cfg.LABELS) else f"cls{cls_id}"
            color = (0, 0, 255) if cls_id == cfg.CONTAINER_CLASS_ID else (255, 0, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame, f"{label} {score:.2f}", (x1, max(y1 - 8, 10)),
                font, 0.4, color, 2,
            )

        if self._best_gate and self.guidance:
            target_pt = (int(self.guidance.target_x), int(self.guidance._current_target_y))
            gate_pt = (int(self._best_gate.cx), int(self._best_gate.cy))
            cv2.arrowedLine(frame, target_pt, gate_pt, (0, 255, 255), 2, tipLength=0.15)

        # HUD panel
        panel = np.zeros((200, 340, 3), dtype=np.uint8)
        y = 18
        avg_infer = (
            self._infer_time_sum / self._infer_time_count
            if self._infer_time_count > 0 else 0.0
        )
        lines = [
            f"KP2026 GATE MISSION v4 | State: {self.state}",
            f"FPS: {self._fps:.0f} | Infer: {avg_infer:.0f}ms",
            f"Gates: {self.gates_passed}/{self.gate_count_target} | "
            f"Tracks: {len(tracks)} | Raw: {len(raw_detections)}",
            f"Time: {time.perf_counter() - self.mission_start_time:.0f}s | "
            f"Tilt: {cfg.CAMERA_TILT_DEG:.0f}deg",
        ]
        if self._best_gate:
            target = self.guidance.gate_to_target(self._best_gate)
            lines += [
                f"Gate: {target.est_distance:.1f}m | conf={self._best_gate.confidence:.2f}",
                f"BBox: {self._best_gate.width:.0f}x{self._best_gate.height:.0f}px "
                f"({target.bbox_ratio * 100:.0f}%)",
                f"Offset: ({target.offset_x_px:.0f}, {target.offset_y_px:.0f})px",
            ]
            if self._last_cmd:
                lines.append(
                    f"CMD: vx={self._last_cmd.vx:.2f} vy={self._last_cmd.vy:.2f} vz={self._last_cmd.vz:.2f}"
                )
        if self.drone:
            alt = self.drone.get_altitude()
            if alt is not None:
                lines.append(f"Alt: {alt:.1f}m")

        for i, line in enumerate(lines):
            cv2.putText(panel, line, (10, y + i * 17), font, 0.38, (255, 255, 255), 1)

        px1, py1, px2, py2 = 5, 5, 345, 205
        if py2 <= h and px2 <= w:
            roi = frame[py1:py2, px1:px2]
            blended = cv2.addWeighted(panel, 0.55, roi, 0.45, 0)
            frame[py1:py2, px1:px2] = blended

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def _cleanup(self):
        logging.info("Cleaning up...")
        if self.depth_monitor:
            try:
                self.depth_monitor.stop()
            except Exception:
                pass
        if self.detector:
            try:
                self.detector.close()
            except Exception:
                pass
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        if self.drone:
            try:
                self.drone.hover()
                time.sleep(0.3)
                self.drone.close()
            except Exception:
                pass
        if self._video_writer:
            self._video_writer.release()
        cv2.destroyAllWindows()
        logging.info("Cleanup complete.")


# ═══════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════


def main():
    import math

    parser = argparse.ArgumentParser(description="KP2026 Gate Mission v4.0")
    parser.add_argument("--gates", type=int, default=1, help="Jumlah gate (default: 1)")
    parser.add_argument("--alt", type=float, default=cfg.TAKEOFF_ALTITUDE, help="Ketinggian takeoff (m)")
    parser.add_argument("--port", type=str, default=cfg.SERIAL_PORT, help="Serial port FC")
    parser.add_argument("--baud", type=int, default=cfg.SERIAL_BAUD, help="Baud rate")
    parser.add_argument("--vision-only", action="store_true", help="Hanya vision, tanpa drone")
    parser.add_argument("--dry-run", action="store_true", help="Testing mode: print command, TIDAK kirim ke hardware")
    parser.add_argument("--conf", type=float, default=cfg.CONF_THRESHOLD, help="Confidence threshold")
    parser.add_argument("--gate-width", type=float, default=cfg.GATE_REAL_WIDTH_M, help="Lebar gate asli (meter)")
    parser.add_argument("--gate-height", type=float, default=cfg.GATE_REAL_HEIGHT_M, help="Tinggi gate asli (meter)")
    parser.add_argument("--tilt", type=float, default=cfg.CAMERA_TILT_DEG, help="Kamera tilt ke atas (derajat)")
    parser.add_argument("--hef", type=str, default=cfg.HEF_PATH, help="Path ke file .hef")
    parser.add_argument("--hud", type=str, choices=["full", "minimal", "none"], default="minimal", help="HUD mode")
    parser.add_argument("--record", action="store_true", help="Record video to file")
    parser.add_argument("--speed-min", type=float, default=cfg.CRUISE_SPEED_MIN,
                        help="Kecepatan minimum saat gate jauh (m/s)")
    parser.add_argument("--speed-max", type=float, default=cfg.CRUISE_SPEED_MAX,
                        help="Kecepatan maksimum saat gate dekat (m/s)")
    parser.add_argument("--pass-speed", type=float, default=cfg.PASS_SPEED,
                        help="Kecepatan saat menerobos gate (m/s)")
    parser.add_argument("--no-depth", action="store_true",
                        help="Disable D435i depth safety (anti-collision)")
    parser.add_argument("--replay", type=str, default=None,
                        help="Replay from video file (png/jpg/mp4/avi)")
    args = parser.parse_args()

    cfg.HEF_PATH = args.hef
    cfg.SERIAL_PORT = args.port
    cfg.SERIAL_BAUD = args.baud

    print("""
    ╔══════════════════════════════════════════════════════════════╗
    ║     KP2026 GATE MISSION v4.0 — MODULAR ARCHITECTURE        ║
    ║  ┌────────────────────────────────────────────────────┐     ║
    ║  │ Camera:  RealSense D435i RGB + Stereo Depth       │     ║
    ║  │ AI:      Hailo-8 YOLO (core/hailo_detector.py)    │     ║
    ║  │ Tracker: IoU + EMA (core/tracker.py)               │     ║
    ║  │ Guidance: Visual BBox + Tilt Comp (core/guidance)  │     ║
    ║  │ Safety:  Depth Anti-Collision (core/depth_safety)  │     ║
    ║  │ FC:      ArduPilot MAVLink (core/drone.py)         │     ║
    ║  │ Config:  config.py                                 │     ║
    ║  └────────────────────────────────────────────────────┘     ║
    ╚══════════════════════════════════════════════════════════════╝
    """)
    print(f"Gate size:   {args.gate_width:.1f}x{args.gate_height:.1f}m")
    print(f"Camera tilt: {args.tilt:.0f}° upward")
    print(f"Confidence:  {args.conf:.2f}")
    print(f"Takeoff alt: {args.alt:.1f}m")
    print(f"Speed:       min={args.speed_min:.1f} max={args.speed_max:.1f} pass={args.pass_speed:.1f} m/s")
    depth_status = "DISABLED" if args.no_depth else "ENABLED"
    print(f"Depth safety:{depth_status} (stereo anti-collision)")
    if args.replay:
        print(f"Replay:      {args.replay}")
    elif args.dry_run:
        print("Mode:        DRY-RUN (commands printed, NOT sent)")
    elif args.vision_only:
        print("Mode:        VISION-ONLY (no drone)")
    else:
        print(f"Drone:       {args.port}@{args.baud}")
    print()

    mission = GateMission(
        takeoff_alt=args.alt,
        gate_count=args.gates,
        vision_only=args.vision_only,
        dry_run=args.dry_run,
        conf_threshold=args.conf,
        gate_width_m=args.gate_width,
        gate_height_m=args.gate_height,
        camera_tilt_deg=args.tilt,
        hud_mode=args.hud,
        record=args.record,
        speed_min=args.speed_min,
        speed_max=args.speed_max,
        pass_speed=args.pass_speed,
        depth_enabled=not args.no_depth,
        replay_video=args.replay,
    )
    mission.run()


if __name__ == "__main__":
    main()
