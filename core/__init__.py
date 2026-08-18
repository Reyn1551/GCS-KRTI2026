"""KP2026 core library — shared utilities for Hailo inference, vision, guidance, drone control, depth safety."""

from core.hailo_detector import HailoDetector  # noqa: F401
from core.tracker import SimpleTracker  # noqa: F401
from core.gate_kf import GateKF  # noqa: F401
from core.guidance import GateDetection, VisualTarget, VelocityCommand, VisualGuidance  # noqa: F401
from core.drone import DroneController, MockDrone  # noqa: F401
from core.camera import load_camera_calibration  # noqa: F401
from core.depth_safety import (  # noqa: F401
    ApertureDetector,
    ClearanceResult,
    DepthSafetyMonitor,
    SafetyState,
    create_monitor,
)
from core.fusion import DepthFusion, FusedTarget  # noqa: F401
