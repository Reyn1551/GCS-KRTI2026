#!/usr/bin/env python3
"""
KP2026 — D435i Depth Viewer (standalone, tanpa Hailo/AI).

Menampilkan:
- Depth colormap dengan forward ROI overlay
- Forward path clearance status
- Depth histogram + noise level
- Measured distance di center frame

Berguna untuk validasi depth pipeline sebelum integrasi misi penuh.

Usage:
    python3 tools/depth_viewer.py
    python3 tools/depth_viewer.py --roi 64 --min-clearance 0.5
"""

import argparse
import logging
import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np

import config as cfg
from core.depth_safety import DepthSafetyMonitor, SafetyState

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s: %(message)s",
)


def main():
    parser = argparse.ArgumentParser(description="KP2026 D435i Depth Viewer")
    parser.add_argument("--roi", type=int, default=cfg.DEPTH_FORWARD_ROI_SIZE,
                        help="Forward ROI size (px, default: 48)")
    parser.add_argument("--min-clearance", type=float, default=cfg.DEPTH_MIN_CLEARANCE_M,
                        help="Minimum clearance for path (m, default: 0.5)")
    parser.add_argument("--emergency", type=float, default=cfg.DEPTH_FORWARD_EMERGENCY_M,
                        help="Emergency threshold (m, default: 0.3)")
    parser.add_argument("--fps", type=int, default=cfg.DEPTH_FPS,
                        help="Depth FPS (default: 30)")
    args = parser.parse_args()

    monitor = DepthSafetyMonitor(
        depth_fps=args.fps,
        forward_roi_size=args.roi,
        min_clearance_m=args.min_clearance,
        emergency_m=args.emergency,
    )

    if not monitor.start(external=False):
        print("Failed to start depth pipeline. Is RealSense D435i connected?")
        return 1

    print("\nD435i Depth Viewer — press 'q' to quit, 'r' to reset\n")

    _perf_counter = time.perf_counter
    fps = 0.0
    prev_t = _perf_counter()
    frame_count = 0

    try:
        while monitor.is_running:
            state = monitor.get_state()
            if not state.depth_available:
                time.sleep(0.05)
                continue

            frame_count += 1
            t_now = _perf_counter()
            dt = t_now - prev_t
            prev_t = t_now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)

            colormap = monitor.get_depth_colormap()
            raw_depth = monitor.get_depth_frame()

            if colormap is None or raw_depth is None:
                continue

            display = np.copy(colormap)
            h, w = display.shape[:0] if display.ndim == 2 else display.shape[:2]

            # ── HUD overlay ───────────────────────────────────────────────
            font = cv2.FONT_HERSHEY_SIMPLEX

            # Status bar di atas
            status_color = (0, 255, 0) if state.forward_clear else (0, 0, 255)
            status_text = (
                f"CLEAR {state.forward_depth:.1f}m"
                if state.forward_clear
                else f"BLOCKED {state.forward_depth:.1f}m"
            )
            cv2.putText(display, status_text, (8, 18), font, 0.5, status_color, 1)

            fps_text = f"FPS:{fps:.0f} Noise:{state.noise_level:.3f}m Dist:{state.measured_distance:.1f}m"
            cv2.putText(display, fps_text, (8, 38), font, 0.4, (255, 255, 255), 1)

            # Warning jika noise tinggi
            if state.noise_level > cfg.DEPTH_NOISE_WARN_THRESHOLD:
                cv2.putText(
                    display, "HIGH NOISE", (w - 120, 18),
                    font, 0.45, (0, 200, 255), 1,
                )

            # Forward ROI label
            cv2.putText(
                display, "FWD-ROI", (w // 2 - 30, h // 2 + 20),
                font, 0.35, (255, 255, 0), 1,
            )

            # Gate clearance (jika ada bbox)
            gc = state.gate_clearance
            if gc is not None and gc.warning:
                gc_color = (0, 255, 0) if gc.is_clear else (0, 140, 255)
                cv2.putText(
                    display, f"GATE: {gc.warning[:50]}", (8, h - 10),
                    font, 0.35, gc_color, 1,
                )

            # ── Depth histogram (bottom-left corner) ──────────────────────
            valid = raw_depth[(raw_depth > 0) & (raw_depth < 15000)]
            if len(valid) > 100:
                hist_h = 60
                hist_w = 200
                hist_x = 8
                hist_y = h - hist_h - 18

                bins = np.linspace(0, 10000, 50)
                hist, edges = np.histogram(valid, bins=bins)
                hist_max = max(hist.max(), 1)
                hist_norm = (hist / hist_max * hist_h).astype(np.int32)

                # Background
                cv2.rectangle(
                    display,
                    (hist_x, hist_y),
                    (hist_x + hist_w, hist_y + hist_h),
                    (30, 30, 30), -1,
                )

                for i in range(len(hist_norm)):
                    bar_h = hist_norm[i]
                    if bar_h > 0:
                        x = hist_x + int(i * hist_w / len(hist_norm))
                        bar_w = max(1, hist_w // len(hist_norm))
                        cv2.rectangle(
                            display,
                            (x, hist_y + hist_h - bar_h),
                            (x + bar_w, hist_y + hist_h),
                            (100, 180, 100), -1,
                        )

                # Mark thresholds
                for thresh_m, thresh_color, thresh_label in [
                    (args.emergency, (0, 0, 255), "EMG"),
                    (args.min_clearance, (0, 255, 255), "MIN"),
                ]:
                    thresh_px = int(thresh_m * 1000)
                    thresh_x = hist_x + int((thresh_px / 10000) * hist_w)
                    if 0 <= thresh_x < hist_x + hist_w:
                        cv2.line(
                            display,
                            (thresh_x, hist_y),
                            (thresh_x, hist_y + hist_h),
                            thresh_color, 1,
                        )
                        cv2.putText(
                            display, thresh_label,
                            (thresh_x + 2, hist_y + 10),
                            font, 0.3, thresh_color, 1,
                        )

                cv2.putText(
                    display, "Depth histogram", (hist_x, hist_y - 4),
                    font, 0.3, (150, 150, 150), 1,
                )

            # ── Center marker ─────────────────────────────────────────────
            cv2.drawMarker(
                display, (w // 2, h // 2),
                (255, 255, 255), cv2.MARKER_CROSS, 12, 1,
            )

            cv2.imshow("D435i Depth Viewer — Anti-Collision", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                logging.info("Reset...")

    except KeyboardInterrupt:
        pass
    finally:
        monitor.stop()
        cv2.destroyAllWindows()
        logging.info("Depth viewer stopped. Frames: %d", frame_count)

    return 0


if __name__ == "__main__":
    sys.exit(main())
