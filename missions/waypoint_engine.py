#!/usr/bin/env python3
"""
KP2026 — WAYPOINT VISION ENGINE v2.0
Hailo YOLO + ArUco Pose + RealSense RGB + Guidance Library

Importable as a library (by web/gcs_web.py) or standalone CLI for debugging.
"""

import argparse
import json
import logging
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
import pyrealsense2 as rs

import config as cfg
from core.camera import load_camera_calibration
from core.hailo_detector import HailoDetector

# ═══════════════════════════════════════════════════════════════════════════
# DATA STRUCTURES
# ═══════════════════════════════════════════════════════════════════════════


@dataclass
class WaypointDetection:
    x1: float; y1: float; x2: float; y2: float; confidence: float
    cx: float = 0.0; cy: float = 0.0; width: float = 0.0; height: float = 0.0

    def __post_init__(self):
        self.cx = (self.x1 + self.x2) / 2.0
        self.cy = (self.y1 + self.y2) / 2.0
        self.width = self.x2 - self.x1
        self.height = self.y2 - self.y1


@dataclass
class ArUcoPose:
    distance: float; yaw_deg: float; pitch_deg: float; roll_deg: float
    tvec: Tuple[float, float, float]; rvec: Tuple[float, float, float]
    marker_id: int; offset_x_px: float; offset_y_px: float; confidence: float


@dataclass
class WPGuidance:
    bearing_deg: float; elevation_deg: float; distance_m: float
    altitude_diff_m: float; offset_x_m: float
    has_aruco: bool; confidence: float


@dataclass
class EngineState:
    frame_bgr: Optional[np.ndarray] = None
    frame_jpeg: Optional[bytes] = None
    detections: List[Dict] = None
    best_wp: Optional[WaypointDetection] = None
    last_aruco: Optional[ArUcoPose] = None
    guidance: Optional[WPGuidance] = None
    fps: float = 0.0; infer_ms: float = 0.0; frame_count: int = 0
    running: bool = False; source: str = ""

    def __post_init__(self):
        if self.detections is None:
            self.detections = []


# ═══════════════════════════════════════════════════════════════════════════
# ARUCO POSE ESTIMATOR
# ═══════════════════════════════════════════════════════════════════════════


class ArUcoEstimator:
    def __init__(self, fx, fy, cx, cy, dist_coeffs, marker_size_m=cfg.ARUCO_MARKER_SIZE_M):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.marker_size_m = marker_size_m
        self.cam_mat = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self.dist = np.array(dist_coeffs, dtype=np.float64).reshape(-1, 1)
        self.target_y = cy + fy * math.tan(cfg.CAMERA_TILT_RAD)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.params = cv2.aruco.DetectorParameters()
        self.params.adaptiveThreshWinSizeMin = 3
        self.params.adaptiveThreshWinSizeMax = 23
        self.params.adaptiveThreshWinSizeStep = 10
        self.params.minMarkerPerimeterRate = 0.03
        self.params.maxMarkerPerimeterRate = 4.0
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.params)

    def detect(self, frame_gray: np.ndarray) -> List[ArUcoPose]:
        corners, ids, _ = self.detector.detectMarkers(frame_gray)
        if ids is None or len(ids) == 0:
            return []
        results = []
        for i, mid in enumerate(ids.flatten()):
            corner = corners[i].reshape((4, 2))
            half = self.marker_size_m / 2.0
            obj = np.array([
                [-half, half, 0], [half, half, 0],
                [half, -half, 0], [-half, -half, 0],
            ], dtype=np.float64)
            ok, rvec, tvec = cv2.solvePnP(obj, corner.astype(np.float64), self.cam_mat, self.dist)
            if not ok:
                continue
            dist = float(np.linalg.norm(tvec))
            tx, ty, tz = tvec.flatten()
            yaw = math.degrees(math.atan2(tx, tz)) if tz > 0.01 else 0.0
            elev = math.degrees(math.atan2(ty, tz)) if tz > 0.01 else 0.0
            elev += cfg.CAMERA_TILT_DEG
            img_pts, _ = cv2.projectPoints(
                np.array([[0, 0, 0]], dtype=np.float64), rvec, tvec, self.cam_mat, self.dist
            )
            cx_m, cy_m = img_pts[0][0]
            results.append(ArUcoPose(
                distance=dist, yaw_deg=yaw, pitch_deg=elev, roll_deg=0.0,
                tvec=(float(tx), float(ty), float(tz)),
                rvec=(float(rvec[0]), float(rvec[1]), float(rvec[2])),
                marker_id=int(mid),
                offset_x_px=float(cx_m - self.cx),
                offset_y_px=float(cy_m - self.target_y),
                confidence=1.0,
            ))
        results.sort(key=lambda p: p.distance)
        return results


# ═══════════════════════════════════════════════════════════════════════════
# WAYPOINT VISUAL GUIDANCE
# ═══════════════════════════════════════════════════════════════════════════


class WPVisualGuidance:
    def __init__(self, fx, fy, cx, cy):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.target_y = cy + fy * math.tan(cfg.CAMERA_TILT_RAD)
        self.target_y_fwd = cy + fy * math.tan(math.radians(5.0))
        self._cur_y = self.target_y

    def estimate_dist(self, w_px: float) -> float:
        if w_px < 1:
            return 99.0
        return max(0.3, min(99.0, (cfg.WP_REAL_WIDTH_M * self.fx) / w_px))

    def from_yolo(self, wp: WaypointDetection) -> WPGuidance:
        d = self.estimate_dist(wp.width)
        ox_px = wp.cx - self.cx
        oy_px = wp.cy - self._cur_y
        return WPGuidance(
            bearing_deg=math.degrees(math.atan2(ox_px * d / self.fx, d)) if d > 0.1 else 0.0,
            elevation_deg=math.degrees(math.atan2(oy_px * d / self.fy, d)) if d > 0.1 else 0.0,
            distance_m=d,
            altitude_diff_m=oy_px * d / self.fy if d > 0.1 else 0.0,
            offset_x_m=ox_px * d / self.fx if d > 0.1 else 0.0,
            has_aruco=False, confidence=wp.confidence,
        )

    def from_aruco(self, pose: ArUcoPose) -> WPGuidance:
        tx, ty, tz = pose.tvec
        dist_h = math.sqrt(tx * tx + tz * tz)
        return WPGuidance(
            bearing_deg=math.degrees(math.atan2(tx, max(tz, 0.01))),
            elevation_deg=math.degrees(math.atan2(ty, max(tz, 0.01))) + cfg.CAMERA_TILT_DEG,
            distance_m=dist_h, altitude_diff_m=ty, offset_x_m=tx,
            has_aruco=True, confidence=pose.confidence,
        )

    def set_forward(self, enabled: bool):
        t = self.target_y_fwd if enabled else self.target_y
        self._cur_y = self._cur_y * 0.7 + t * 0.3


# ═══════════════════════════════════════════════════════════════════════════
# WAYPOINT ENGINE
# ═══════════════════════════════════════════════════════════════════════════


class WaypointEngine:
    def __init__(
        self,
        replay_path: Optional[str] = None,
        conf_threshold: float = cfg.CONF_THRESHOLD,
        camera_tilt_deg: float = cfg.CAMERA_TILT_DEG,
        on_frame: Optional[Callable] = None,
    ):
        cfg.CAMERA_TILT_DEG = camera_tilt_deg
        cfg.CAMERA_TILT_RAD = math.radians(camera_tilt_deg)
        cfg.CONF_THRESHOLD = conf_threshold

        self.replay_path = replay_path
        self.on_frame = on_frame
        self.state = EngineState()
        self.state.source = "Replay" if replay_path else "RealSense"

        self.detector: Optional[HailoDetector] = None
        self.aruco: Optional[ArUcoEstimator] = None
        self.guidance: Optional[WPVisualGuidance] = None
        self.pipeline: Optional[rs.pipeline] = None
        self.video_cap: Optional[cv2.VideoCapture] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self):
        fx, fy, cx, cy, dc = load_camera_calibration(cfg.CALIB_PATH)

        self.detector = HailoDetector(cfg.HEF_PATH, conf_thres=cfg.CONF_THRESHOLD)
        self.aruco = ArUcoEstimator(fx, fy, cx, cy, dc, camera_tilt_deg=cfg.CAMERA_TILT_DEG)
        self.guidance = WPVisualGuidance(fx, fy, cx, cy)

        if self.replay_path:
            self.video_cap = cv2.VideoCapture(self.replay_path)
        else:
            self.pipeline = rs.pipeline()
            conf = rs.config()
            conf.enable_stream(
                rs.stream.color, cfg.CAMERA_WIDTH, cfg.CAMERA_HEIGHT,
                rs.format.bgr8, cfg.CAMERA_FPS,
            )
            self.pipeline.start(conf)
            for _ in range(10):
                self.pipeline.wait_for_frames()

        logging.info("WaypointEngine: started")
        self.state.running = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)
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
        if self.video_cap:
            self.video_cap.release()
        self.state.running = False
        logging.info("WaypointEngine: stopped")

    def _loop(self):
        _perf = time.perf_counter
        _last_wp_time = _perf()
        fps = 0.0
        prev_t = _perf()
        frame_count = 0

        while not self._stop.is_set():
            if self.pipeline:
                frames = self.pipeline.wait_for_frames()
                cf = frames.get_color_frame()
                if not cf:
                    continue
                frame = np.asanyarray(cf.get_data())
            elif self.video_cap:
                ret, frame = self.video_cap.read()
                if not ret:
                    logging.info("WaypointEngine: replay ended")
                    break
            else:
                break

            t0 = _perf()
            dt = t0 - prev_t
            prev_t = t0
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)
            frame_count += 1

            detections = self.detector.infer(frame)

            best = None
            best_score = 0.0
            for d in detections:
                if d["class_id"] == cfg.WAYPOINT_CLASS_ID and d["score"] > best_score:
                    best_score = d["score"]
                    best = d

            best_wp: Optional[WaypointDetection] = None
            if best and best_score >= cfg.MIN_CONFIDENCE_WP:
                x1, y1, x2, y2 = best["bbox"]
                best_wp = WaypointDetection(x1=x1, y1=y1, x2=x2, y2=y2, confidence=best_score)
                _last_wp_time = t0

            last_aruco: Optional[ArUcoPose] = None
            if t0 - _last_wp_time > cfg.LOST_WP_TIMEOUT:
                best_wp = None

            # ArUco detection
            try:
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                aruco_poses = self.aruco.detect(frame_gray)
                if aruco_poses:
                    last_aruco = aruco_poses[0]
            except Exception:
                pass

            guidance: Optional[WPGuidance] = None
            if last_aruco:
                guidance = self.guidance.from_aruco(last_aruco)
            elif best_wp:
                guidance = self.guidance.from_yolo(best_wp)

            if best_wp:
                self.guidance.set_forward(best_wp.width > cfg.CAMERA_WIDTH * 0.20)

            infer_ms = (_perf() - t0) * 1000.0

            self.state.detections = detections
            self.state.best_wp = best_wp
            self.state.last_aruco = last_aruco
            self.state.guidance = guidance
            self.state.fps = fps
            self.state.infer_ms = infer_ms
            self.state.frame_count = frame_count
            self.state.frame_bgr = frame

            _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            self.state.frame_jpeg = jpeg.tobytes()

            if self.on_frame:
                try:
                    self.on_frame(frame, {
                        "detections": detections, "guidance": guidance,
                        "fps": fps, "infer_ms": infer_ms, "frame_count": frame_count,
                    })
                except Exception:
                    pass

        self.state.running = False
        self.state.frame_bgr = None


# ═══════════════════════════════════════════════════════════════════════════
# OVERLAY HELPER
# ═══════════════════════════════════════════════════════════════════════════


def draw_overlay(frame: np.ndarray, state: EngineState) -> np.ndarray:
    h, w = frame.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX

    for det in state.detections:
        cls_id, score = det["class_id"], det["score"]
        x1, y1, x2, y2 = det["bbox"]
        if cls_id == cfg.WAYPOINT_CLASS_ID:
            color, label = (255, 165, 0), f"WP {score:.2f}"
        elif cls_id == cfg.GATE_CLASS_ID:
            color, label = (0, 255, 0), f"GATE {score:.2f}"
        elif cls_id == cfg.CONTAINER_CLASS_ID:
            color, label = (0, 0, 255), f"BOX {score:.2f}"
        else:
            color, label = (200, 200, 200), f"cls{cls_id} {score:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, max(y1 - 6, 12)), font, 0.4, color, 1)

    cx_t = int(cfg.CX)
    cy_t = int(cfg.CY + cfg.FY * math.tan(cfg.CAMERA_TILT_RAD))
    cv2.line(frame, (0, cy_t), (w, cy_t), (100, 200, 255), 1)
    cv2.drawMarker(frame, (cx_t, cy_t), (0, 200, 255), cv2.MARKER_CROSS, 18, 2)

    if state.best_wp:
        wp = state.best_wp
        cx_wp, cy_wp = int(wp.cx), int(wp.cy)
        cv2.arrowedLine(frame, (cx_t, cy_t), (cx_wp, cy_wp), (255, 165, 0), 2, tipLength=0.12)
        cv2.circle(frame, (cx_wp, cy_wp), 4, (255, 165, 0), -1)

    if state.last_aruco and state.frame_bgr is not None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        aruco_params = cv2.aruco.DetectorParameters()
        aruco_det = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, _ = aruco_det.detectMarkers(gray)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # Mini radar
    if state.guidance:
        g = state.guidance
        rad_cx, rad_cy = w - 80, h - 80
        rad_r = 55
        cv2.circle(frame, (rad_cx, rad_cy), rad_r, (30, 30, 30), -1)
        cv2.circle(frame, (rad_cx, rad_cy), rad_r, (80, 80, 80), 1)
        for r in [14, 28, 42]:
            cv2.circle(frame, (rad_cx, rad_cy), r, (50, 50, 50), 1)
        cv2.line(frame, (rad_cx - rad_r, rad_cy), (rad_cx + rad_r, rad_cy), (50, 50, 50), 1)
        cv2.line(frame, (rad_cx, rad_cy - rad_r), (rad_cx, rad_cy + rad_r), (50, 50, 50), 1)
        cv2.circle(frame, (rad_cx, rad_cy), 3, (0, 255, 0), -1)
        br = math.radians(-g.bearing_deg)
        max_d = max(10.0, min(g.distance_m, 20.0))
        dot_dist = int((max_d / 20.0) * rad_r * 0.8)
        dx = rad_cx + int(dot_dist * math.sin(br))
        dy = rad_cy + int(dot_dist * math.cos(br))
        dx = max(rad_cx - rad_r + 5, min(rad_cx + rad_r - 5, dx))
        dy = max(rad_cy - rad_r + 5, min(rad_cy + rad_r - 5, dy))
        cd = (0, 200, 255) if g.has_aruco else (255, 165, 0)
        cv2.circle(frame, (dx, dy), 5, cd, -1)
        cv2.circle(frame, (dx, dy), 5, (255, 255, 255), 1)
        cv2.putText(frame, f"{g.distance_m:.0f}m", (rad_cx - rad_r, rad_cy + rad_r + 14), font, 0.33, (180, 180, 180), 1)
        cv2.putText(frame, f"{g.bearing_deg:+.0f}°", (rad_cx + rad_r + 4, rad_cy + 4), font, 0.33, (180, 180, 180), 1)

    lines = [f"FPS: {state.fps:.0f} | Infer: {state.infer_ms:.0f}ms | Frame: {state.frame_count}"]
    if state.guidance:
        g = state.guidance
        src = "ArUco" if g.has_aruco else "YOLO"
        lines += [f"D: {g.distance_m:.1f}m | B: {g.bearing_deg:+.0f}° | {src}"]
        lines += [f"X: {g.offset_x_m:+.2f}m | Alt: {g.altitude_diff_m:+.2f}m"]
        if g.has_aruco and state.last_aruco:
            a = state.last_aruco
            lines += [f"ArUco #{a.marker_id} Yaw: {a.yaw_deg:+.0f}° Dist: {a.distance:.1f}m"]
    else:
        lines.append("No waypoint detected")

    for i, line in enumerate(lines):
        cv2.putText(frame, line, (8, 18 + i * 16), font, 0.37, (50, 220, 255), 1)

    return frame


# ═══════════════════════════════════════════════════════════════════════════
# STANDALONE CLI
# ═══════════════════════════════════════════════════════════════════════════


def main_cli():
    parser = argparse.ArgumentParser(description="KP2026 Waypoint Vision Engine — CLI debug")
    parser.add_argument("--replay", type=str, default=None, help="Video file replay")
    parser.add_argument("--conf", type=float, default=cfg.CONF_THRESHOLD, help="Confidence threshold")
    parser.add_argument("--tilt", type=float, default=cfg.CAMERA_TILT_DEG, help="Camera tilt degrees")
    parser.add_argument("--hef", type=str, default=cfg.HEF_PATH, help="Path ke .hef")
    parser.add_argument("--record", action="store_true", help="Record output video")
    args = parser.parse_args()

    cfg.HEF_PATH = args.hef

    logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s")

    engine = WaypointEngine(
        replay_path=args.replay, conf_threshold=args.conf, camera_tilt_deg=args.tilt,
    )

    vid_writer = None
    if args.record:
        os.makedirs(cfg.LOG_DIR, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        vid_writer = cv2.VideoWriter(
            os.path.join(cfg.LOG_DIR, f"vision_{ts}.avi"), fourcc, 15.0, (640, 480),
        )

    print("Standalone vision engine. Press 'q' to quit.")
    engine.start()
    try:
        while engine.state.running:
            frame = engine.state.frame_bgr
            if frame is None:
                time.sleep(0.01)
                continue
            display = draw_overlay(frame.copy(), engine.state)
            if vid_writer:
                vid_writer.write(display)
            cv2.imshow("KP2026 Waypoint Vision Engine", display)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        engine.stop()
        if vid_writer:
            vid_writer.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main_cli()
