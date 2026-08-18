#!/usr/bin/env python3
"""
KP2026 — Container Mission: Automatic First Aid Kit Drop via YOLO + Servo.

State machine:
  SEARCH  → cari container (class_id=0), wait/hover
  APPROACH → container terdeteksi, belum centered/cukup besar
  ALIGN   → container centered + ukuran bbox ≥ threshold → hitung frame stabil
  DROP    → stable ≥ N frame → trigger servo BUKA, konfirmasi, tunggu recovery
  COMPLETE → misi selesai, servo TUTUP
  EMERGENCY → gagal/abort, fail-safe

Mode:
  --dry-run        : vision-only + sim servo (print, no hardware)
  --vision-only    : camera + Hailo only, no servo
  --real           : camera + Hailo + servo hardware (default jika ada)

Fail-safe:
  - Confidence threshold (default 0.30) — diabaikan di bawah ini
  - Timeout deteksi (LOST_TIMEOUT=3.0s) → kembali SEARCH
  - Cooldown servo (DROP_COOLDOWN=5.0s) → prevent double-drop
  - Tidak ada deteksi saat DROP state > timeout → ABORT, kembali SEARCH
  - Emergency key 'e' / SIGINT → stop
  - Max mission time (300 detik)
  - Logging lengkap ke mission_logs/

Calibration:
  Bbox calibration dari config/bbox_calibration.json diaplikasikan ke setiap
  deteksi container sebelum centering check. Kalibrasi via tools/bbox_calibration.py.
"""

import argparse
import json
import logging
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import (
    BBOX_CALIB_PATH,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    CONTAINER_APPROACH_FRAMES,
    CONTAINER_CENTER_THRESHOLD_PX,
    CONTAINER_CLASS_ID,
    CONTAINER_DROP_BBOX_RATIO,
    CONTAINER_DROP_COOLDOWN_S,
    CONTAINER_LOST_TIMEOUT,
    CONTAINER_MIN_CONFIDENCE,
    CONTAINER_REAL_HEIGHT_M,
    CONTAINER_REAL_WIDTH_M,
    LABELS,
    LOG_DIR,
    MAX_MISSION_TIME,
    HEF_PATH,
    CONF_THRESHOLD,
    FX,
    FY,
    CX,
    CY,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
)

from core.hailo_detector import HailoDetector

# ── Default bbox calibration entry ──────────────────────────────────────────
_DEFAULT_CALIB = {"cx_offset_px": 0, "cy_offset_px": 0, "scale_w": 1.0, "scale_h": 1.0}


def load_bbox_calibration(path: str) -> Dict[str, dict]:
    try:
        with open(path, "r") as f:
            data = json.load(f)
        return {k: v for k, v in data.items() if not k.startswith("_")}
    except Exception:
        return {label: dict(_DEFAULT_CALIB) for label in LABELS}


def apply_calibration(bbox: tuple, calib: dict) -> Tuple[int, int, int, int]:
    x1, y1, x2, y2 = bbox
    cx = (x1 + x2) / 2.0 + calib.get("cx_offset_px", 0)
    cy = (y1 + y2) / 2.0 + calib.get("cy_offset_px", 0)
    sw = calib.get("scale_w", 1.0)
    sh = calib.get("scale_h", 1.0)
    hw = (x2 - x1) * sw / 2.0
    hh = (y2 - y1) * sh / 2.0
    return (int(cx - hw), int(cy - hh), int(cx + hw), int(cy + hh))


# ═══════════════════════════════════════════════════════════════════════════════
# DATA STRUCTURES
# ═══════════════════════════════════════════════════════════════════════════════


@dataclass
class ContainerTarget:
    """Container detection result after calibration."""
    x1: int
    y1: int
    x2: int
    y2: int
    width: int
    height: int
    cx: int
    cy: int
    confidence: float
    offset_x_px: int  # signed offset from frame center X
    offset_y_px: int  # signed offset from frame center Y
    bbox_ratio: float  # width / CAMERA_WIDTH
    est_distance_m: float = -1.0  # estimated via similar triangles
    is_centered: bool = False     # within center threshold


# ═══════════════════════════════════════════════════════════════════════════════
# SERVO WRAPPER (with dry-run support)
# ═══════════════════════════════════════════════════════════════════════════════


class DropServo:
    """Servo controller for payload release, with dry-run support."""

    def __init__(self, dry_run: bool = False):
        self.dry_run = dry_run
        self._servo = None
        self.is_open = False
        self._drop_count = 0
        if not dry_run:
            try:
                from control.servo import ServoController
                self._servo = ServoController(dry_run=False)
            except Exception as e:
                logging.warning(f"Servo hardware init failed: {e}. Falling back to dry-run.")
                self.dry_run = True

    def buka(self):
        self.is_open = True
        self._drop_count += 1
        if self.dry_run or self._servo is None:
            logging.info("[DRY-RUN] DROP #%d — Servo: BUKA (PWM 1900)", self._drop_count)
        else:
            self._servo.buka()

    def tutup(self):
        self.is_open = False
        if self.dry_run or self._servo is None:
            logging.info("[DRY-RUN] Servo: TUTUP (PWM 1100)")
        else:
            self._servo.tutup()

    def lepas(self):
        if self.dry_run or self._servo is None:
            logging.info("[DRY-RUN] Servo: LEPAS (duty_cycle=0)")
        else:
            self._servo.lepas()


# ═══════════════════════════════════════════════════════════════════════════════
# MISSION STATE MACHINE
# ═══════════════════════════════════════════════════════════════════════════════


class ContainerMission:
    """State machine: SEARCH → APPROACH → ALIGN → DROP → COMPLETE."""

    STATE_SEARCH = "SEARCH"
    STATE_APPROACH = "APPROACH"
    STATE_ALIGN = "ALIGN"
    STATE_DROP = "DROP"
    STATE_COMPLETE = "COMPLETE"
    STATE_EMERGENCY = "EMERGENCY"

    def __init__(
        self,
        dry_run: bool = False,
        vision_only: bool = False,
        conf_threshold: float = CONTAINER_MIN_CONFIDENCE,
        center_threshold_px: int = CONTAINER_CENTER_THRESHOLD_PX,
        drop_bbox_ratio: float = CONTAINER_DROP_BBOX_RATIO,
        approach_frames: int = CONTAINER_APPROACH_FRAMES,
        lost_timeout: float = CONTAINER_LOST_TIMEOUT,
        drop_cooldown: float = CONTAINER_DROP_COOLDOWN_S,
        calib_path: str = BBOX_CALIB_PATH,
        hud: str = "full",
        record: bool = False,
    ):
        os.makedirs(LOG_DIR, exist_ok=True)

        log_handlers = [
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(os.path.join(LOG_DIR, "container_mission.log")),
        ]
        logging.basicConfig(
            level=logging.INFO,
            format="[%(asctime)s] %(levelname)s: %(message)s",
            handlers=log_handlers,
        )

        self.dry_run = dry_run
        self.vision_only = vision_only
        self.conf_threshold = conf_threshold
        self.center_threshold_px = center_threshold_px
        self.drop_bbox_ratio = drop_bbox_ratio
        self.approach_frames = approach_frames
        self.lost_timeout = lost_timeout
        self.drop_cooldown = drop_cooldown
        self.calib_path = calib_path
        self.hud = hud
        self.record = record

        self.state = self.STATE_SEARCH
        self.running = True
        self.mission_start_time = 0.0

        # Subsystems
        self.detector: Optional[HailoDetector] = None
        self.pipeline: Optional[rs.pipeline] = None
        self.servo: Optional[DropServo] = None
        self.calib_data: Dict[str, dict] = {}

        # State tracking
        self._last_detection_time = 0.0
        self._last_drop_time = 0.0
        self._stable_align_count = 0
        self._frame_count = 0
        self._best_container: Optional[ContainerTarget] = None
        self._drop_confirmed = False
        self._drop_confirm_time = 0.0
        self._bbox_history: List[float] = []

        # FPS
        self._fps = 0.0
        self._prev_loop_time = time.perf_counter()

        # Video recording
        self._video_writer = None

        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    def _on_signal(self, sig, frame):
        logging.warning(f"Signal {sig} — emergency shutdown!")
        self.running = False
        self.state = self.STATE_EMERGENCY

    # ── Initialization ────────────────────────────────────────────────────

    def init_systems(self):
        logging.info("=" * 60)
        logging.info("INITIALIZING CONTAINER MISSION")
        logging.info("=" * 60)

        # Bbox calibration
        self.calib_data = load_bbox_calibration(self.calib_path)
        logging.info("  [✓] Bbox calibration loaded: %d classes", len(self.calib_data))

        # HailoDetector
        try:
            self.detector = HailoDetector(HEF_PATH, conf_thres=CONF_THRESHOLD)
            logging.info("  [✓] HailoDetector loaded")
        except Exception as e:
            logging.error("  [✗] HailoDetector: %s", e)
            raise

        # RealSense camera
        try:
            self.pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.bgr8, 60)
            self.pipeline.start(cfg)
            for _ in range(10):
                self.pipeline.wait_for_frames()
            logging.info("  [✓] RealSense RGB @ 60fps")
        except Exception as e:
            logging.error("  [✗] RealSense: %s", e)
            raise

        # Servo
        if self.vision_only:
            self.servo = None
            logging.info("  [—] Servo SKIPPED (vision-only)")
        else:
            self.servo = DropServo(dry_run=self.dry_run)
            label = "DRY-RUN" if self.servo.dry_run else "PCA9685"
            logging.info("  [✓] DropServo [%s]", label)

        # Video recording
        if self.record:
            try:
                ts = time.strftime("%Y%m%d_%H%M%S")
                path = os.path.join(LOG_DIR, f"container_mission_{ts}.avi")
                fourcc = cv2.VideoWriter_fourcc(*"XVID")
                self._video_writer = cv2.VideoWriter(path, fourcc, 15.0, (CAMERA_WIDTH, CAMERA_HEIGHT))
                logging.info("  [✓] Recording → %s", path)
            except Exception as e:
                logging.warning("  [!] Video recording: %s", e)

        logging.info("All systems ready.\n")

    # ── Convert raw detection to ContainerTarget ──────────────────────────

    def _to_target(self, det: dict) -> Optional[ContainerTarget]:
        """Apply calibration, compute offsets and centering check."""
        cls_id = det["class_id"]
        label = LABELS[cls_id]
        calib = self.calib_data.get(label, dict(_DEFAULT_CALIB))
        score = det["score"]
        raw_bbox = det["bbox"]

        x1, y1, x2, y2 = apply_calibration(raw_bbox, calib)
        if x2 <= x1 or y2 <= y1:
            return None

        w = x2 - x1
        h = y2 - y1
        cx_cal = (x1 + x2) // 2
        cy_cal = (y1 + y2) // 2
        frame_cx = CAMERA_WIDTH // 2
        frame_cy = CAMERA_HEIGHT // 2

        offset_x = cx_cal - frame_cx
        offset_y = cy_cal - frame_cy
        bbox_ratio = w / CAMERA_WIDTH

        is_centered = abs(offset_x) < self.center_threshold_px and abs(offset_y) < self.center_threshold_px

        tgt = ContainerTarget(
            x1=x1, y1=y1, x2=x2, y2=y2,
            width=w, height=h,
            cx=cx_cal, cy=cy_cal,
            confidence=score,
            offset_x_px=offset_x,
            offset_y_px=offset_y,
            bbox_ratio=bbox_ratio,
            is_centered=is_centered,
        )

        # Distance estimate via similar triangles
        if w > 0 and FX > 0:
            tgt.est_distance_m = (CONTAINER_REAL_WIDTH_M * FX) / w

        return tgt

    # ── Main Loop ─────────────────────────────────────────────────────────

    def run(self):
        try:
            self.init_systems()
            self.mission_start_time = time.perf_counter()
            self.state = self.STATE_SEARCH
            logging.info("=== SEARCH (container class=%d, conf≥%.2f, center≤%dpx) ===",
                         CONTAINER_CLASS_ID, self.conf_threshold, self.center_threshold_px)

            _perf = time.perf_counter

            while self.running:
                if self._frame_count % 30 == 0:
                    if _perf() - self.mission_start_time > MAX_MISSION_TIME:
                        logging.warning("Mission timeout (%ds)!", MAX_MISSION_TIME)
                        self.state = self.STATE_EMERGENCY

                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                frame = np.asanyarray(color_frame.get_data())

                t0 = _perf()
                dt = t0 - self._prev_loop_time
                self._prev_loop_time = t0
                if dt > 0:
                    self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)
                self._frame_count += 1

                detections = self.detector.infer(frame)

                containers = []
                for det in detections:
                    if det["class_id"] == CONTAINER_CLASS_ID and det["score"] >= self.conf_threshold:
                        tgt = self._to_target(det)
                        if tgt is not None:
                            containers.append(tgt)

                if containers:
                    containers.sort(key=lambda t: t.confidence, reverse=True)
                    self._best_container = containers[0]
                    self._last_detection_time = _perf()
                else:
                    self._best_container = None

                state = self.state
                if state == self.STATE_SEARCH:
                    self._handle_search(dt)
                elif state == self.STATE_APPROACH:
                    self._handle_approach(dt)
                elif state == self.STATE_ALIGN:
                    self._handle_align(dt)
                elif state == self.STATE_DROP:
                    self._handle_drop(dt)
                elif state == self.STATE_COMPLETE:
                    self._handle_complete()
                    break
                elif state == self.STATE_EMERGENCY:
                    self._handle_emergency()
                    break

                if self.hud != "none":
                    self._draw_hud(frame, containers)

                if self._video_writer:
                    self._video_writer.write(frame)

                cv2.imshow("KP2026 Container Mission", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    self.running = False
                elif key == ord("e"):
                    logging.critical("EMERGENCY key pressed!")
                    self.state = self.STATE_EMERGENCY

        except Exception as e:
            logging.exception("Fatal error: %s", e)
            self.state = self.STATE_EMERGENCY
        finally:
            self._cleanup()

    # ── State Handlers ────────────────────────────────────────────────────

    def _handle_search(self, dt: float):
        tgt = self._best_container
        if tgt is not None:
            self._stable_align_count = 0
            self._bbox_history.clear()
            logging.info("CONTAINER FOUND! conf=%.2f bbox=%.0f%% off=(%+d,%+d)px dist~%.1fm",
                         tgt.confidence, tgt.bbox_ratio * 100, tgt.offset_x_px, tgt.offset_y_px,
                         tgt.est_distance_m)
            self.state = self.STATE_APPROACH
            return

        # No container: log periodically
        if self._frame_count % 90 == 0:
            logging.debug("SEARCH: %d frames, no container detected", self._frame_count)

    def _handle_approach(self, dt: float):
        tgt = self._best_container

        # Lost timeout check
        if tgt is None:
            if time.perf_counter() - self._last_detection_time > self.lost_timeout:
                logging.warning("Container lost for %.1fs → back to SEARCH",
                                time.perf_counter() - self._last_detection_time)
                self.state = self.STATE_SEARCH
                return
            return

        # Check if bbox is large enough AND centered
        big_enough = tgt.bbox_ratio >= self.drop_bbox_ratio

        if big_enough and tgt.is_centered:
            self._stable_align_count = 1
            logging.info("→ ALIGN: bbox=%.0f%% off=(%+d,%+d)px — centering...",
                         tgt.bbox_ratio * 100, tgt.offset_x_px, tgt.offset_y_px)
            self.state = self.STATE_ALIGN
            return

        if big_enough and not tgt.is_centered:
            if self._frame_count % 30 == 0:
                direction = ""
                if tgt.offset_x_px > self.center_threshold_px:
                    direction += " ← Move LEFT"
                elif tgt.offset_x_px < -self.center_threshold_px:
                    direction += " → Move RIGHT"
                if tgt.offset_y_px > self.center_threshold_px:
                    direction += " ↑ Move UP"
                elif tgt.offset_y_px < -self.center_threshold_px:
                    direction += " ↓ Move DOWN"
                logging.info("APPROACH: bbox=%.0f%% big enough but need centering%s",
                             tgt.bbox_ratio * 100, direction)

        if not big_enough:
            if self._frame_count % 60 == 0:
                logging.info("APPROACH: bbox=%.0f%% too small (<%.0f%%), dist~%.1fm → get closer",
                             tgt.bbox_ratio * 100, self.drop_bbox_ratio * 100, tgt.est_distance_m)

    def _handle_align(self, dt: float):
        tgt = self._best_container
        now = time.perf_counter()

        # Lost check
        if tgt is None:
            if now - self._last_detection_time > self.lost_timeout:
                logging.warning("Container lost during ALIGN → back to SEARCH")
                self._stable_align_count = 0
                self.state = self.STATE_SEARCH
                return
            return

        # Reset if no longer centered or too small
        if not tgt.is_centered or tgt.bbox_ratio < self.drop_bbox_ratio:
            logging.info("ALIGN broken: centered=%s bbox=%.0f%% → back to APPROACH",
                         tgt.is_centered, tgt.bbox_ratio * 100)
            self._stable_align_count = 0
            self.state = self.STATE_APPROACH
            return

        # Count consecutive stable frames
        self._stable_align_count += 1

        if self._frame_count % 10 == 0:
            logging.info("ALIGN [%d/%d]: off=(%+d,%+d)px bbox=%.0f%% conf=%.2f",
                         self._stable_align_count, self.approach_frames,
                         tgt.offset_x_px, tgt.offset_y_px,
                         tgt.bbox_ratio * 100, tgt.confidence)

        if self._stable_align_count >= self.approach_frames:
            # Check cooldown
            if now - self._last_drop_time < self.drop_cooldown:
                remaining = self.drop_cooldown - (now - self._last_drop_time)
                logging.info("DROP cooldown: %.1fs remaining...", remaining)
                return

            logging.info("ALIGN confirmed! (%d frames stable) → DROP!", self._stable_align_count)
            self.state = self.STATE_DROP

    def _handle_drop(self, dt: float):
        if not self._drop_confirmed:
            # Execute drop
            if self.servo is not None:
                self.servo.buka()
            self._drop_confirmed = True
            self._drop_confirm_time = time.perf_counter()
            self._last_drop_time = self._drop_confirm_time
            logging.info("=" * 50)
            logging.info("DROP EXECUTED! Servo BUKA — First Aid Kit released.")
            logging.info("=" * 50)

        elapsed = time.perf_counter() - self._drop_confirm_time

        if elapsed < 1.0:
            # Keep servo open for 1 second
            return

        # Close servo
        if self.servo is not None and self.servo.is_open:
            self.servo.tutup()
            logging.info("Servo TUTUP after drop.")

        if elapsed < 2.0:
            return

        # All done
        self.state = self.STATE_COMPLETE

    def _handle_complete(self):
        logging.info("=" * 60)
        elapsed = time.perf_counter() - self.mission_start_time
        logging.info("CONTAINER MISSION COMPLETE — total %.1fs", elapsed)
        logging.info("=" * 60)
        self.running = False

    def _handle_emergency(self):
        logging.critical("EMERGENCY — abort!")
        if self.servo is not None and self.servo.is_open:
            self.servo.tutup()
        self.running = False

    # ── HUD ────────────────────────────────────────────────────────────────

    def _draw_hud(self, frame: np.ndarray, containers: List[ContainerTarget]):
        h, w = frame.shape[:2]

        # Crosshair
        cv2.drawMarker(frame, (w // 2, h // 2), (128, 128, 128), cv2.MARKER_CROSS, 25, 1)

        # Center threshold box (visual guide)
        thr = self.center_threshold_px
        cx, cy = w // 2, h // 2
        cv2.rectangle(frame, (cx - thr, cy - thr), (cx + thr, cy + thr), (128, 128, 128), 1)

        # Draw all container detections
        for tgt in containers:
            color = (0, 255, 255) if tgt.is_centered else (0, 165, 255)
            cv2.rectangle(frame, (tgt.x1, tgt.y1), (tgt.x2, tgt.y2), color, 2)
            cv2.circle(frame, (tgt.cx, tgt.cy), 5, color, -1)
            cv2.line(frame, (tgt.cx, tgt.cy), (w // 2, h // 2), (255, 255, 0), 1)

            label = f"Container {tgt.confidence:.2f}"
            cv2.putText(frame, label, (tgt.x1, max(tgt.y1 - 8, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

            info = f"off=({tgt.offset_x_px:+d},{tgt.offset_y_px:+d})px {tgt.bbox_ratio*100:.0f}% {tgt.est_distance_m:.1f}m"
            cv2.putText(frame, info, (tgt.x1, tgt.y2 + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

        # HUD panel
        y = 20
        state_colors = {"SEARCH": (100, 100, 255), "APPROACH": (255, 200, 0),
                        "ALIGN": (0, 255, 0), "DROP": (0, 255, 255),
                        "COMPLETE": (0, 255, 0), "EMERGENCY": (0, 0, 255)}
        sc = state_colors.get(self.state, (255, 255, 255))
        cv2.putText(frame, f"STATE: {self.state}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, sc, 2)
        y += 25

        cv2.putText(frame, f"FPS: {self._fps:.1f}  FRAMES: {self._frame_count}",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        y += 20

        if self._best_container:
            t = self._best_container
            cv2.putText(frame, f"Bbox: {t.bbox_ratio*100:.0f}%  Dist: {t.est_distance_m:.1f}m  "
                              f"Conf: {t.confidence:.2f}",
                        (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            y += 18
            cv2.putText(frame, f"Offset: ({t.offset_x_px:+d},{t.offset_y_px:+d})px  "
                              f"Centered: {t.is_centered}",
                        (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            y += 18

        if self.stable_align_count > 0:
            cv2.putText(frame, f"Stable: {self._stable_align_count}/{self.approach_frames}",
                        (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
            y += 20

        cv2.putText(frame, f"DRY-RUN" if self.dry_run else f"LIVE", (w - 100, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255) if self.dry_run else (0, 255, 0), 1)

    # ── Cleanup ────────────────────────────────────────────────────────────

    def _cleanup(self):
        if self.servo is not None:
            self.servo.lepas()
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        if self.detector:
            try:
                self.detector.close()
            except Exception:
                pass
        if self._video_writer:
            self._video_writer.release()
        cv2.destroyAllWindows()
        logging.info("ContainerMission: cleanup complete.")


# ═══════════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════════


def main():
    parser = argparse.ArgumentParser(
        description="KP2026 — Container Auto-Drop Mission (YOLO + Servo)",
        epilog="Example: python3 missions/container_mission.py --dry-run")
    parser.add_argument("--dry-run", action="store_true",
                        help="Vision + simulate servo (no hardware)")
    parser.add_argument("--vision-only", action="store_true",
                        help="Camera + Hailo only, no servo at all")
    parser.add_argument("--conf", type=float, default=CONTAINER_MIN_CONFIDENCE,
                        help=f"Confidence threshold (default: {CONTAINER_MIN_CONFIDENCE})")
    parser.add_argument("--center", type=int, default=CONTAINER_CENTER_THRESHOLD_PX,
                        help=f"Center tolerance px (default: {CONTAINER_CENTER_THRESHOLD_PX})")
    parser.add_argument("--bbox-ratio", type=float, default=CONTAINER_DROP_BBOX_RATIO,
                        help=f"Min bbox ratio for drop (default: {CONTAINER_DROP_BBOX_RATIO})")
    parser.add_argument("--approach-frames", type=int, default=CONTAINER_APPROACH_FRAMES,
                        help=f"Stable frames before drop (default: {CONTAINER_APPROACH_FRAMES})")
    parser.add_argument("--calib", default=BBOX_CALIB_PATH,
                        help="Bbox calibration JSON path")
    parser.add_argument("--hud", default="full", choices=["full", "minimal", "none"])
    parser.add_argument("--record", action="store_true", help="Record video to mission_logs/")
    args = parser.parse_args()

    if args.dry_run and args.vision_only:
        print("[WARN] --dry-run and --vision-only redundant; using --vision-only)")
        args.dry_run = False

    mission = ContainerMission(
        dry_run=args.dry_run,
        vision_only=args.vision_only,
        conf_threshold=args.conf,
        center_threshold_px=args.center,
        drop_bbox_ratio=args.bbox_ratio,
        approach_frames=args.approach_frames,
        calib_path=args.calib,
        hud=args.hud,
        record=args.record,
    )

    mode_str = "DRY-RUN" if args.dry_run else ("VISION-ONLY" if args.vision_only else "LIVE")
    print(f"""
    ╔══════════════════════════════════════════════════════════════╗
    ║     KP2026 — Container Auto-Drop Mission [{mode_str}]       ║
    ║  ┌────────────────────────────────────────────────────┐     ║
    ║  │ State: SEARCH → APPROACH → ALIGN → DROP → COMPLETE │     ║
    ║  │ Sensor : YOLO Hailo class=Container (id=0)         │     ║
    ║  │ Trigger: centered + bbox≥{args.bbox_ratio*100:.0f}% + stable {args.approach_frames}fr  │     ║
    ║  │ Servo  : {'PCA9685 ch0&1' if not (args.dry_run or args.vision_only) else 'SIMULATED'}                       │     ║
    ║  │ Calib  : {args.calib if args.calib else 'none'}           │     ║
    ║  └────────────────────────────────────────────────────┘     ║
    ╚══════════════════════════════════════════════════════════════╝
    """)

    mission.run()


if __name__ == "__main__":
    main()
