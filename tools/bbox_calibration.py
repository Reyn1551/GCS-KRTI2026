#!/usr/bin/env python3
"""
KP2026 — Interactive Bounding Box Calibration Tool.

Load Hailo YOLO model + RealSense camera. Overlay detections. Adjust per-
class bbox center offsets and scale factors with keyboard, review instantly,
then save to config/bbox_calibration.json for use by auto-drop missions.

Controls:
  Arrow keys            → move selected class center offset (1 px step)
  Shift + Arrow keys    → move 10 px step
  Tab                   → cycle selected class (Container / gate / Waypoint)
  +/-                   → scale width/height of selected class bbox
  r                     → reset selected class calibration to zero
  s                     → save calibration to JSON
  q / Esc               → quit
  h                     → toggle HUD
  f                     → freeze/unfreeze frame (inspect)
  Space                 → toggle crosshair overlay
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import (
    BBOX_CALIB_PATH,
    LABELS,
)
from core.hailo_detector import HailoDetector

# ── Default offsets applied per class ─────────────────────────────────────
_DEFAULT_CALIB = {"cx_offset_px": 0, "cy_offset_px": 0, "scale_w": 1.0, "scale_h": 1.0}


def load_calibration(path: str) -> dict:
    try:
        with open(path, "r") as f:
            data = json.load(f)
        return {k: v for k, v in data.items() if not k.startswith("_")}
    except Exception as e:
        print(f"[WARN] Cannot load calibration: {e}. Using defaults.")
        return {}


def save_calibration(path: str, data: dict):
    payload = {"_version": 1, "_description": "Per-class bounding box calibration", **data}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\n[OK] Calibration saved → {path}")


def apply_calibration(bbox: tuple, calib: dict) -> tuple:
    """Apply center offset and scale to a bbox (x1,y1,x2,y2)."""
    x1, y1, x2, y2 = bbox
    cx = (x1 + x2) / 2.0 + calib.get("cx_offset_px", 0)
    cy = (y1 + y2) / 2.0 + calib.get("cy_offset_px", 0)
    sw = calib.get("scale_w", 1.0)
    sh = calib.get("scale_h", 1.0)
    hw = (x2 - x1) * sw / 2.0
    hh = (y2 - y1) * sh / 2.0
    return (int(cx - hw), int(cy - hh), int(cx + hw), int(cy + hh))


def main():
    parser = argparse.ArgumentParser(description="KP2026 — Bbox Calibration Tool")
    parser.add_argument("--hef", default=None, help="Path to HEF model")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--calib", default=BBOX_CALIB_PATH, help="Calibration JSON path")
    parser.add_argument("--hud", default="full", choices=["full", "minimal", "none"],
                        help="HUD mode")
    args = parser.parse_args()

    hef_path = args.hef
    if hef_path is None:
        default_hef = os.path.join(os.path.dirname(os.path.dirname(__file__)), "model", "320px-v1.hef")
        hef_path = default_hef if os.path.exists(default_hef) else None
    if hef_path is None or not os.path.exists(hef_path):
        print("[ERROR] HEF model not found. Use --hef <path>")
        sys.exit(1)

    calib_data = load_calibration(args.calib)
    for cls_name in LABELS:
        if cls_name not in calib_data:
            calib_data[cls_name] = dict(_DEFAULT_CALIB)

    selected_idx = 0
    selected_class = LABELS[selected_idx]
    frozen = False
    frozen_frame = None
    show_hud = args.hud != "none"
    show_crosshair = True
    step_px = 1

    # ── Init Hailo ──────────────────────────────────────────────────────
    print(f"Loading HEF: {hef_path}")
    detector = HailoDetector(hef_path, conf_thres=args.conf)

    # ── Init RealSense ──────────────────────────────────────────────────
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
    pipeline.start(cfg)
    for _ in range(10):
        pipeline.wait_for_frames()

    print("""
╔══════════════════════════════════════════════════════════════╗
║   KP2026 — Bounding Box Calibration Tool                    ║
║                                                            ║
║   Arrow keys → offset center (±1 px)                       ║
║   Shift+Arrow → offset center (±10 px)                     ║
║   Tab        → cycle class (Container/gate/Waypoint)        ║
║   +/-        → scale bbox width/height                      ║
║   r          → reset selected class                         ║
║   s          → save to JSON                                 ║
║   f          → freeze/unfreeze frame                        ║
║   Space      → toggle crosshair                             ║
║   h          → toggle HUD                                   ║
║   q / Esc    → quit                                         ║
╚══════════════════════════════════════════════════════════════╝
""")

    try:
        while True:
            if not frozen:
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                frame = np.asanyarray(color_frame.get_data())
            else:
                frame = frozen_frame.copy()

            detections = detector.infer(frame)

            # ── Draw raw (uncalibrated) detections ────────────────────────
            for det in detections:
                cls_id = det["class_id"]
                score = det["score"]
                if cls_id >= len(LABELS):
                    continue
                label = LABELS[cls_id]
                calib = calib_data.get(label, dict(_DEFAULT_CALIB))

                x1, y1, x2, y2 = det["bbox"]
                raw_color = (100, 100, 100)  # grey = raw

                cv2.rectangle(frame, (x1, y1), (x2, y2), raw_color, 1)
                cv2.putText(frame, f"raw {label} {score:.2f}",
                            (x1, max(y1 - 8, 0)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.38, raw_color, 1)

                # ── Draw calibrated bbox ─────────────────────────────────
                cx1, cy1, cx2, cy2 = apply_calibration((x1, y1, x2, y2), calib)
                cal_color = (0, 255, 255) if label == selected_class else (0, 255, 0)

                cv2.rectangle(frame, (cx1, cy1), (cx2, cy2), cal_color, 2)
                # Draw calibrated center as dot
                ccx = (cx1 + cx2) // 2
                ccy = (cy1 + cy2) // 2
                cv2.circle(frame, (ccx, ccy), 4, cal_color, -1)

                if show_hud or label == selected_class:
                    offset_info = (f"off=({calib['cx_offset_px']:+d},{calib['cy_offset_px']:+d}) "
                                   f"sc=({calib['scale_w']:.1f},{calib['scale_h']:.1f})")
                    cv2.putText(frame, f"{label} {score:.2f} {offset_info}",
                                (cx1, max(cy1 - 12, 0)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.4, cal_color, 1)

            # ── Crosshair ────────────────────────────────────────────────
            if show_crosshair:
                h, w = frame.shape[:2]
                cx, cy = w // 2, h // 2
                cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 30, 1)
                cv2.line(frame, (0, cy), (w, cy), (0, 0, 255), 1)
                cv2.line(frame, (cx, 0), (cx, h), (0, 0, 255), 1)

            # ── HUD panel ────────────────────────────────────────────────
            if show_hud:
                y = 30
                for i, name in enumerate(LABELS):
                    cal = calib_data.get(name, dict(_DEFAULT_CALIB))
                    marker = " >>> " if name == selected_class else "     "
                    color = (0, 255, 255) if name == selected_class else (200, 200, 200)
                    text = (f"{marker}{name}: cx={cal['cx_offset_px']:+4d} "
                            f"cy={cal['cy_offset_px']:+4d} "
                            f"sw={cal['scale_w']:.1f} sh={cal['scale_h']:.1f}")
                    cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                                0.45, color, 1)
                    y += 22

                cv2.putText(frame, f"STEP={step_px}px | s=save r=reset q=quit",
                            (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.4, (150, 150, 150), 1)

            cv2.imshow("Bbox Calibration", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break

            cal = calib_data[selected_class]

            if key == ord("\t") or key == 9:  # Tab
                selected_idx = (selected_idx + 1) % len(LABELS)
                selected_class = LABELS[selected_idx]
                print(f"Selected: {selected_class}")

            elif key == 81 or key == ord("a"):  # Left
                cal["cx_offset_px"] -= step_px
            elif key == 83 or key == ord("d"):  # Right
                cal["cx_offset_px"] += step_px
            elif key == 82 or key == ord("w"):  # Up
                cal["cy_offset_px"] -= step_px
            elif key == 84 or key == ord("s"):  # Down
                cal["cy_offset_px"] += step_px

            # Shift+Arrow (uses different keycodes on some systems)
            elif key == 0:
                key2 = cv2.waitKey(1) & 0xFFFF
                if key2 == 0xFF51 or key2 == 0x250000:  # shift+left
                    step_px = 10
                elif key2 == 0xFF53 or key2 == 0x270000:  # shift+right
                    step_px = 10
                elif key2 == 0xFF52 or key2 == 0x260000:  # shift+up
                    step_px = 10
                elif key2 == 0xFF54 or key2 == 0x280000:  # shift+down
                    step_px = 10

            elif key == ord("+"):
                cal["scale_w"] = round(cal["scale_w"] + 0.05, 2)
                cal["scale_h"] = round(cal["scale_h"] + 0.05, 2)
            elif key == ord("-"):
                cal["scale_w"] = round(max(0.05, cal["scale_w"] - 0.05), 2)
                cal["scale_h"] = round(max(0.05, cal["scale_h"] - 0.05), 2)

            elif key == ord("r"):
                calib_data[selected_class] = dict(_DEFAULT_CALIB)
                print(f"[RESET] {selected_class} → {_DEFAULT_CALIB}")

            elif key == ord("s"):
                save_calibration(args.calib, calib_data)

            elif key == ord("f"):
                frozen = not frozen
                if frozen:
                    frozen_frame = frame.copy()
                    print("[FREEZE] frame frozen for inspection")
                else:
                    print("[UNFREEZE] live feed resumed")

            elif key == ord("h"):
                show_hud = not show_hud
                print(f"[HUD] {'ON' if show_hud else 'OFF'}")

            elif key == ord(" "):
                show_crosshair = not show_crosshair
                print(f"[CROSSHAIR] {'ON' if show_crosshair else 'OFF'}")

            # Arrow keycodes via cv2.waitKey
            elif key == ord("j"):
                cal["cx_offset_px"] -= step_px
            elif key == ord("l"):
                cal["cx_offset_px"] += step_px
            elif key == ord("i"):
                cal["cy_offset_px"] -= step_px
            elif key == ord("k"):
                cal["cy_offset_px"] += step_px

    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        detector.close()
        cv2.destroyAllWindows()
        print("\nTool closed.")


if __name__ == "__main__":
    main()
