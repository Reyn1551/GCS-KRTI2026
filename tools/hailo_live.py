#!/usr/bin/env python3
"""
KP2026 — Hailo Live Viewer
Standalone: RealSense RGB → Hailo YOLO → OpenCV display.

Uses core/hailo_detector and core/tracker for shared inference pipeline.
"""

import argparse
import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
import pyrealsense2 as rs

import config as cfg
from core.hailo_detector import HailoDetector
from core.tracker import SimpleTracker

logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s")


def main():
    parser = argparse.ArgumentParser(description="KP2026 Hailo Live Viewer")
    parser.add_argument("--hef", type=str, default=cfg.HEF_PATH, help="Path to HEF model")
    parser.add_argument("--conf", type=float, default=cfg.CONF_THRESHOLD, help="Confidence threshold")
    parser.add_argument("--max-miss", type=int, default=cfg.TRACKER_MAX_MISS, help="Tracker max miss frames")
    parser.add_argument("--smooth", type=float, default=cfg.TRACKER_SMOOTH_ALPHA, help="Tracker smoothing alpha")
    args = parser.parse_args()

    config, pipeline = None, None
    try:
        detector = HailoDetector(args.hef, conf_thres=args.conf)
        tracker = SimpleTracker(max_miss=args.max_miss, smooth_alpha=args.smooth)

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
        pipeline.start(config)
        for _ in range(10):
            pipeline.wait_for_frames()

        logging.info("Live viewer running. Press 'q' to quit.")

        prev_time = time.time()
        fps = 0.0
        frame_count = 0
        infer_sum = 0.0
        infer_count = 0

        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)

            frame = np.asanyarray(color_frame.get_data())
            t0 = time.perf_counter()
            detections = detector.infer(frame)
            infer_ms = (time.perf_counter() - t0) * 1000.0
            infer_sum += infer_ms
            infer_count += 1
            frame_count += 1

            tracks = tracker.update(detections)

            for _tid, bbox, cls_id, score, _age in tracks:
                x1, y1, x2, y2 = bbox
                label = cfg.LABELS[cls_id] if cls_id < len(cfg.LABELS) else f"cls{cls_id}"
                colors = [(0, 0, 255), (0, 255, 0), (255, 165, 0)]
                color = colors[cls_id % 3]

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    frame, f"{label} {score:.2f}",
                    (x1, max(y1 - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1,
                )

            avg_infer = infer_sum / infer_count if infer_count > 0 else 0.0
            cv2.putText(
                frame,
                f"FPS: {fps:.0f} | Inf: {avg_infer:.0f}ms | Det: {len(detections)} | Trk: {len(tracks)}",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1,
            )

            cv2.imshow("KP2026 Hailo Live", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            if frame_count % 60 == 0:
                logging.info(
                    "Frame %d | FPS: %.0f | Inf: %.0fms | Det: %d | Trk: %d",
                    frame_count, fps, avg_infer, len(detections), len(tracks),
                )

    finally:
        if pipeline:
            pipeline.stop()
        if detector:
            detector.close()
        cv2.destroyAllWindows()
        logging.info("Viewer ended. %d frames.", frame_count if "frame_count" in dir() else 0)


if __name__ == "__main__":
    main()
