#!/usr/bin/env bash
# Start both camera streams on the Pi.
#
# First run:  v4l2-ctl --list-devices
# and set the device nodes below. The RealSense D435i exposes several nodes;
# pick the one under "Intel RealSense D435I" that carries the RGB stream
# (the RGB one supports MJPG/YUYV at 640x480 and up).
#
# Install deps once:  sudo apt install python3-opencv v4l2-utils

CAM_DOWN=/dev/video0    # Logitech webcam (down-facing)
CAM_FRONT=/dev/video4   # RealSense D435i RGB (forward) - ADJUST after v4l2-ctl

cd "$(dirname "$0")"

python3 camera_server.py --device "$CAM_DOWN"  --port 8080 --width 640 --height 480 &
python3 camera_server.py --device "$CAM_FRONT" --port 8081 --width 640 --height 480 &

echo "down cam  -> http://$(hostname -I | awk '{print $1}'):8080/stream"
echo "front cam -> http://$(hostname -I | awk '{print $1}'):8081/stream"

wait
