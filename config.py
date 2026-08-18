"""
KP2026 Central Configuration — All constants in one place.

Import this module to access any configuration value.
Runtime overrides via argparse are applied by reassigning module-level attributes.
"""
import math
import os

# ═══════════════════════════════════════════════════════════════════════════
# Paths
# ═══════════════════════════════════════════════════════════════════════════
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
HEF_PATH = os.path.join(BASE_DIR, "model", "320px-v1.hef")
CALIB_PATH = os.path.join(BASE_DIR, "camera_calibration.json")
LOG_DIR = os.path.join(BASE_DIR, "mission_logs")

# ═══════════════════════════════════════════════════════════════════════════
# Model YOLO
# ═══════════════════════════════════════════════════════════════════════════
LABELS = ["Container", "gate", "Waypoint"]
NUM_CLASSES = len(LABELS)
CONTAINER_CLASS_ID = 0
GATE_CLASS_ID = 1
WAYPOINT_CLASS_ID = 2
CONF_THRESHOLD = 0.55
NMS_IOU_THRESHOLD = 0.15

# ═══════════════════════════════════════════════════════════════════════════
# Camera (RealSense D435i RGB)
# ═══════════════════════════════════════════════════════════════════════════
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 60

# Default intrinsics (overridden from camera_calibration.json at runtime)
FX = 592.0
FY = 610.0
CX = 339.0
CY = 243.0

# Camera mount tilt (degrees upward)
CAMERA_TILT_DEG = 0.0
CAMERA_TILT_RAD = math.radians(CAMERA_TILT_DEG)

# ═══════════════════════════════════════════════════════════════════════════
# Gate Physical (from RULES.md)
# ═══════════════════════════════════════════════════════════════════════════
GATE_REAL_WIDTH_M = 1.9
GATE_REAL_HEIGHT_M = 2.0
GATE_OPENING_WIDTH_M = 1.5
GATE_OPENING_HEIGHT_M = 1.5

# ═══════════════════════════════════════════════════════════════════════════
# Waypoint Physical (from RULES.md)
# ═══════════════════════════════════════════════════════════════════════════
WP_REAL_WIDTH_M = 2.0
WP_REAL_HEIGHT_M = 2.0

# ArUco marker physical size (meters)
ARUCO_MARKER_SIZE_M = 0.50
ARUCO_MARKER_SIZE_SMALL_M = 0.10
ARUCO_DICT_ID = 4  # cv2.aruco.DICT_6X6_250
ARUCO_WP_IDS = set(range(10))

# ═══════════════════════════════════════════════════════════════════════════
# MAVLink / Drone Connection
# ═══════════════════════════════════════════════════════════════════════════
SERIAL_PORT = "/dev/ttyAMA0"
SERIAL_BAUD = 921600
MAVLINK_HOST = "127.0.0.1:14551"  # dedicated MAVProxy --out port (GCS keeps 14550)

# ═══════════════════════════════════════════════════════════════════════════
# Movement Strategy (Gate Mission)
# ═══════════════════════════════════════════════════════════════════════════
TAKEOFF_ALTITUDE = 1.6       # m — ketinggian takeoff

# ── Phase Transition Ratios (bbox width / frame width) ──
FAST_TO_PRECISION_RATIO = 0.20
PASS_BBOX_RATIO = 0.65
FORCE_STRAIGHT_RATIO = 0.50  # saat bbox ≥ 50% frame, koreksi lateral = 0, maju lurus

# ── Speed Profile ──
CRUISE_SPEED_MIN = 4.0          # m/s — lambat saat gate jauh (waktu center)
CRUISE_SPEED_MAX = 6.2          # m/s — cepat saat gate dekat (momentum pass)
PASS_SPEED = 2.0                # m/s — lurus melewati gate

# ── Centering Thresholds ──
CENTERING_OFFSET_PX = 30        # px — threshold offset untuk centering
CENTERED_FRAMES_THRESHOLD = 20  # frames — berapa frame harus centered

# ── Lateral Control ──
VY_LIMIT = 2.0                  # max lateral correction
VZ_LIMIT = 1.0

# ═══════════════════════════════════════════════════════════════════════════
# PID — fast phase (aggressive lateral centering)
# ═══════════════════════════════════════════════════════════════════════════
# Gains for NORMALIZED error [-1..1]
KP_X_FAST = 6.0
KI_X_FAST = 1.0
KD_X_FAST = 2.0
KP_Y_FAST = 5.0
KI_Y_FAST = 0.8
KD_Y_FAST = 1.5

# ═══════════════════════════════════════════════════════════════════════════
# PID — precision phase (same or higher — responsive at close range)
# ═══════════════════════════════════════════════════════════════════════════
KP_X_PREC = 5.0
KI_X_PREC = 0.8
KD_X_PREC = 1.5
KP_Y_PREC = 4.0
KI_Y_PREC = 0.6
KD_Y_PREC = 1.2

# ═══════════════════════════════════════════════════════════════════════════
# Timing
# ═══════════════════════════════════════════════════════════════════════════
MIN_APPROACH_FRAMES = 20
PASS_POST_SECONDS = 2.0
LOST_GATE_TIMEOUT = 2.0
TRACKER_MAX_MISS = 8
TRACKER_IOU_THRES = 0.15
TRACKER_SMOOTH_ALPHA = 0.5

# ═══════════════════════════════════════════════════════════════════════════
# Safety
# ═══════════════════════════════════════════════════════════════════════════
MAX_MISSION_TIME = 300
EMERGENCY_ALTITUDE = 2.0
MIN_CONFIDENCE_GATE = 0.25
MIN_CONFIDENCE_WP = 0.25
LOST_WP_TIMEOUT = 2.0

# ═══════════════════════════════════════════════════════════════════════════
# Servo / Payload Release
# ═══════════════════════════════════════════════════════════════════════════
SERVO_NO_TARGET = 10
PWM_BUKA = 1900
PWM_TUTUP = 1100
HEARTBEAT_TIMEOUT_S = 5.0

# ═══════════════════════════════════════════════════════════════════════════
# Depth Safety (RealSense D435i Stereo Depth — anti-collision)
# ═══════════════════════════════════════════════════════════════════════════
DEPTH_ENABLED = True
DEPTH_WIDTH = 424
DEPTH_HEIGHT = 240
DEPTH_FPS = 30
DEPTH_FORWARD_ROI_SIZE = 48
DEPTH_MIN_CLEARANCE_M = 0.5
DEPTH_FORWARD_EMERGENCY_M = 0.3
DEPTH_GATE_CENTER_CLEAR_M = 1.0
DEPTH_EDGE_CLEARANCE_RATIO = 0.20
DEPTH_ASYMMETRY_THRESHOLD_M = 0.3
DEPTH_TEMPORAL_WINDOW = 5
DEPTH_NOISE_WARN_THRESHOLD = 0.15
DEPTH_MEDIAN_MAX_DEPTH = 10.0
DEPTH_PASS_SPEED_MULTIPLIER = 0.5

# ═══════════════════════════════════════════════════════════════════════════
# Container / Drop Target (from RULES.md)
# ═══════════════════════════════════════════════════════════════════════════
# Container / red box real dimensions (meters)
CONTAINER_REAL_WIDTH_M = 0.5
CONTAINER_REAL_HEIGHT_M = 0.4

# Drop centering — seberapa dekat harus aligned sebelum drop
CONTAINER_CENTER_THRESHOLD_PX = 25   # px — offset max dari center frame
CONTAINER_DROP_BBOX_RATIO = 0.20     # bbox width / frame width min untuk drop
CONTAINER_APPROACH_FRAMES = 15       # frame berturut container harus terdeteksi & centered
CONTAINER_LOST_TIMEOUT = 3.0         # detik — timeout container hilang → SEARCH ulang
CONTAINER_MIN_CONFIDENCE = 0.30      # confidence threshold untuk container
CONTAINER_DROP_COOLDOWN_S = 5.0      # detik — cooldown antar trigger servo

# Bbox calibration file (per-class center offsets)
BBOX_CALIB_PATH = os.path.join(BASE_DIR, "config", "bbox_calibration.json")

# ═══════════════════════════════════════════════════════════════════════════
# Display / HUD
# ═══════════════════════════════════════════════════════════════════════════
HUD_MODE = "minimal"
RECORD_VIDEO = False
