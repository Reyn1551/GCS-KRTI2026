#!/usr/bin/env bash
# Start both camera streams on the Pi.
#
# First run:  v4l2-ctl --list-devices
# and set the device nodes below. The RealSense D435i exposes several nodes;
# pick the one under "Intel RealSense D435I" that carries the RGB stream
# (the RGB one supports MJPG/YUYV at 640x480 and up).
#
# Install deps once:  sudo apt install python3-opencv v4l2-utils

cd "$(dirname "$0")"

# Hailo-8 YOLOv26 model for object-detection overlay on the down camera
# (dummy stage: boxes are drawn, no action is taken from the detections).
MODEL="${MODEL:-../model/KP2026V1-YOLOv26.hef}"

# Auto-detect camera devices if not explicitly specified
CAM_DOWN="${CAM_DOWN:-$(python3 camera_server.py --detect down)}"
# RealSense front camera disabled for now
# CAM_FRONT="${CAM_FRONT:-$(python3 camera_server.py --detect front)}"

echo "Auto-detected camera video device indices:"
echo "  Down Cam (Logitech):         $CAM_DOWN"
echo "  Front Cam (RealSense D435i): DISABLED"
echo "  Detect model:                $MODEL"

# RealSense front camera DISABLED for now - only the Logitech down camera runs.
# Re-enable later by uncommenting the CAM_FRONT lines below.
python3 camera_server.py --device "$CAM_DOWN"  --port 8080 --width 640 --height 480 \
    --fps 60 --detect-model "$MODEL" &
# python3 camera_server.py --device "$CAM_FRONT" --port 8081 --width 640 --height 480 &

echo "down cam  -> http://$(hostname -I | awk '{print $1}'):8080/stream"
# echo "front cam -> http://$(hostname -I | awk '{print $1}'):8081/stream"

wait

