#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
# KP2026 Gate Mission — Launch Script v4.0
# ═══════════════════════════════════════════════════════════════════════════
# Refactored structure: uses missions/gate_mission.py with core/ library.
# ═══════════════════════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "========================================="
echo " KP2026 Gate Mission Launcher v4.0"
echo "========================================="
echo ""

# ── Check dependencies ─────────────────────────────────────────────────────
check_dep() {
    python3 -c "import $1" 2>/dev/null && echo "  [✓] $1" || {
        echo "  [✗] $1 NOT FOUND — install with: pip install $2"
        return 1
    }
}

echo "Checking dependencies..."
check_dep "cv2" "opencv-python" || true
check_dep "numpy" "numpy" || true
check_dep "pyrealsense2" "pyrealsense2" || true
check_dep "hailo_platform" "hailort" || true
check_dep "pymavlink" "pymavlink" || true
echo ""

# ── Check hardware ─────────────────────────────────────────────────────────
echo "Checking hardware..."
if [ -e /dev/ttyAMA0 ]; then
    echo "  [✓] Serial port /dev/ttyAMA0 found"
else
    echo "  [!] Warning: /dev/ttyAMA0 not found"
    echo "      → Use --port /dev/ttyUSB0 or --vision-only"
fi

if lsusb 2>/dev/null | grep -qi "Intel.*RealSense"; then
    echo "  [✓] Intel RealSense detected (USB)"
else
    echo "  [!] Warning: RealSense not found on USB"
fi
echo ""

# ── Launch ───────────────────────────────────────────────────────────────
echo "Starting Gate Mission..."
echo "  Controls: 'q' = quit | 'e' = emergency | 'l' = land"
echo "  Mode:    -dr  = dry-run | -v  = vision-only"
echo ""

# Auto-detect MAVProxy — jika MAVProxy sudah running, gunakan UDP proxy
DEFAULT_PORT="/dev/ttyAMA0"
if pgrep -f "mavproxy.py.*--out=udp:127.0.0.1:14550" > /dev/null 2>&1; then
    echo "  [✓] MAVProxy detected → connecting via udpin:127.0.0.1:14550"
    DEFAULT_PORT="udpin:127.0.0.1:14550"
fi
echo ""

# Run from project root with PYTHONPATH set
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"
python3 missions/gate_mission.py \
    --gates 1 \
    --alt 1.4 \
    --tilt 0 \
    --hef model/320px-v1.hef \
    --port "$DEFAULT_PORT" \
    --speed-min 0.5 \
    --speed-max 3.5 \
    --pass-speed 2.0 \
    "$@"


# # Terbang pelan-pelan aman
# ./launch_mission.sh --speed-min 0.4 --speed-max 1.5 --pass-speed 1.5

# # Terbang agresif
# ./launch_mission.sh --speed-min 0.8 --speed-max 3.5 --pass-speed 2.5

# # Hanya ganti pass-speed
# ./launch_mission.sh --pass-speed 1.8
