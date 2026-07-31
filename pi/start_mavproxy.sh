#!/usr/bin/env bash
# Start MAVProxy on the Pi: forward the flight controller to the GCS laptop.
#
# 1) Set GCS_IP to the laptop's IP on the drone WiFi
#    (on Windows run: ipconfig, look for the WiFi adapter IPv4 address)
# 2) Set FC_SERIAL/FC_BAUD to match the flight controller wiring.
# 3) Copy pca_servo.py to ~/.mavproxy/modules/ (or run from this directory).
#
# Install once:
#   pip3 install MAVProxy adafruit-circuitpython-pca9685 adafruit-circuitpython-motor

FC_SERIAL=/dev/serial0
FC_BAUD=921600
GCS_IP=192.168.10.237
    # <-- CHANGE to the GCS laptop IP
GCS_PORT=14550

mavproxy.py \
  --master="$FC_SERIAL" \
  --baudrate="$FC_BAUD" \
  --out=udp:"$GCS_IP":"$GCS_PORT" \
  --load-module=pca_servo \
  --aircraft=drone
