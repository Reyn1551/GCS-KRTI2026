"""
KP2026 Drone Interface — MAVLink controller + MockDrone for dry-run.

Provides a unified interface for real hardware (via pymavlink) and testing.
"""

import logging
import time
from typing import Optional, Union

from config import SERIAL_BAUD, SERIAL_PORT

try:
    from pymavlink import mavutil  # type: ignore[import-untyped]

    HAS_MAVLINK = True
except ImportError:
    mavutil = None  # type: ignore[assignment]
    HAS_MAVLINK = False


# ═══════════════════════════════════════════════════════════════════════════
# Real Drone Controller (MAVLink)
# ═══════════════════════════════════════════════════════════════════════════


# ArduPilot Copter custom mode IDs (fallback jika mode_mapping gagal)
_ARDUCOPTER_MODE_IDS = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3,
    "GUIDED": 4, "LOITER": 5, "RTL": 6, "CIRCLE": 7,
    "LAND": 9, "DRIFT": 11, "SPORT": 13, "FLIP": 14,
    "AUTOTUNE": 15, "POSHOLD": 16, "BRAKE": 17,
    "THROW": 18, "AVOID_ADSB": 19, "GUIDED_NOGPS": 20,
    "SMART_RTL": 21, "FLOWHOLD": 22, "FOLLOW": 23,
    "ZIGZAG": 24, "SYSTEMID": 25, "AUTOROTATE": 26, "AUTO_RTL": 27,
}


class DroneController:
    """MAVLink interface to ArduPilot flight controller."""

    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD):
        if not HAS_MAVLINK:
            raise ImportError("pymavlink not installed. Use --dry-run or --vision-only.")
        self.master = None
        self.target_system = 1
        self.target_component = 1
        self.armed = False
        self._mode_mapping = {}
        self._connect(port, baud)

    def _connect(self, port: str, baud: int):
        logging.info("DroneController: Connecting %s@%d...", port, baud)
        self.master = mavutil.mavlink_connection(
            device=port, baud=baud, source_system=255
        )
        msg = self.master.wait_heartbeat(timeout=10)
        if msg is None:
            raise ConnectionError("No heartbeat from flight controller!")
        self.target_system = msg.get_srcSystem()
        self.target_component = msg.get_srcComponent()
        logging.info("DroneController: Connected sys=%d", self.target_system)

        self._load_mode_mapping()

    def _load_mode_mapping(self):
        try:
            self._mode_mapping = self.master.mode_mapping()
            if self._mode_mapping:
                logging.info("DroneController: Mode mapping loaded from FC")
                return
        except Exception:
            pass
        self._mode_mapping = _ARDUCOPTER_MODE_IDS
        logging.info("DroneController: Using fallback ArduCopter mode IDs")

    def set_mode(self, mode: str, verify: bool = True) -> bool:
        mode_id = self._mode_mapping.get(mode.upper())
        if mode_id is None:
            logging.error("Unknown mode: %s (available: %s)", mode, list(self._mode_mapping.keys())[:10])
            return False
        logging.info("DroneController: Setting mode %s (id=%d)...", mode.upper(), mode_id)
        self.master.mav.set_mode_send(
            self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        time.sleep(0.5)
        if verify:
            if not self._verify_mode(mode.upper(), mode_id, timeout=3.0):
                logging.error("DroneController: Failed to confirm mode change to %s", mode.upper())
                return False
        return True

    def _verify_mode(self, mode_name: str, expected_id: int, timeout: float = 3.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg:
                actual_mode = mavutil.mode_string_v10(msg)
                actual_id = msg.custom_mode
                logging.info("DroneController: Current mode = %s (id=%d)", actual_mode, actual_id)
                if actual_id == expected_id:
                    return True
        return False

    def arm(self) -> bool:
        self.master.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        timeout_t = time.time() + 10
        while time.time() < timeout_t:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                self.armed = True
                logging.info("DroneController: Armed.")
                return True
        logging.error("DroneController: Arm timeout!")
        return False

    def disarm(self):
        self.master.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        self.armed = False

    def takeoff(self, altitude: float) -> bool:
        logging.info("DroneController: Starting takeoff sequence to %.1fm", altitude)
        if not self.set_mode("GUIDED"):
            logging.error("DroneController: Cannot switch to GUIDED. Aborting takeoff.")
            return False
        if not self.arm():
            logging.error("DroneController: Arming failed. Check pre-arm checks on FC.")
            return False
        time.sleep(0.5)
        self.master.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude,
        )
        logging.info("DroneController: Takeoff command sent, waiting for altitude...")
        target_alt = altitude * 0.95
        timeout_t = time.time() + 30
        while time.time() < timeout_t:
            msg = self.master.recv_match(
                type="GLOBAL_POSITION_INT", blocking=True, timeout=2
            )
            if msg and (msg.relative_alt / 1000.0) >= target_alt:
                logging.info("DroneController: Takeoff complete at %.1fm.", altitude)
                return True
            time.sleep(0.5)
        logging.warning("DroneController: Takeoff timeout (continuing anyway).")
        return True

    def send_velocity_body(self, vx: float, vy: float, vz: float):
        """Send velocity command in body-frame NED."""
        IGNORE = 0b00000001 | 0b00000010 | 0b00000100 | 0b01000000 | 0b10000000 | 0b100000000 | 0b1000000000
        self.master.mav.set_position_target_local_ned_send(
            0, self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            IGNORE,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0,
        )

    def send_yaw_rate(self, rate: float):
        IGNORE = 0b0000000011111111111
        self.master.mav.set_position_target_local_ned_send(
            0, self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            IGNORE,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, rate,
        )

    def hover(self):
        self.send_velocity_body(0, 0, 0)

    def land(self):
        self.set_mode("LAND")

    def rtl(self):
        self.set_mode("RTL")

    def get_altitude(self) -> Optional[float]:
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        return msg.relative_alt / 1000.0 if msg else None

    def close(self):
        if self.master:
            self.master.close()


# ═══════════════════════════════════════════════════════════════════════════
# Mock Drone (dry-run / testing)
# ═══════════════════════════════════════════════════════════════════════════


class MockDrone:
    """Mock drone for bench testing — prints all commands instead of sending."""

    def __init__(self, takeoff_alt: float = 3.0):
        self._altitude = 0.0
        self._takeoff_target = takeoff_alt
        self._last_cmd = "HOVER"
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_vz = 0.0
        self._last_yaw_rate = 0.0
        self.armed = False
        self._log_counter = 0
        logging.info("MockDrone: DRY-RUN — commands printed, NOT sent to hardware.")

    def set_mode(self, mode: str) -> bool:
        logging.info("  [DRY-RUN] set_mode(%s)", mode)
        return True

    def arm(self) -> bool:
        self.armed = True
        logging.info("  [DRY-RUN] ARM")
        return True

    def disarm(self):
        self.armed = False
        logging.info("  [DRY-RUN] DISARM")

    def takeoff(self, altitude: float) -> bool:
        self._takeoff_target = altitude
        self._altitude = altitude
        logging.info("  [DRY-RUN] TAKEOFF → %.1fm", altitude)
        self._last_cmd = f"TAKEOFF {altitude}m"
        return True

    def send_velocity_body(self, vx: float, vy: float, vz: float):
        self._last_cmd = "VELOCITY"
        self._last_vx = vx
        self._last_vy = vy
        self._last_vz = vz
        self._log_counter += 1
        if self._log_counter % 30 == 0:
            logging.info(
                "  [DRY-RUN] VELOCITY → vx=%+.2f vy=%+.2f vz=%+.2f m/s",
                vx, vy, vz,
            )

    def send_yaw_rate(self, rate: float):
        self._last_cmd = f"YAW {rate:+.2f} rad/s"
        self._last_yaw_rate = rate

    def hover(self):
        self._last_cmd = "HOVER"
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_vz = 0.0

    def land(self):
        logging.info("  [DRY-RUN] LAND")
        self._last_cmd = "LAND"
        self._altitude = 0.0

    def rtl(self):
        logging.info("  [DRY-RUN] RTL (Return to Launch)")
        self._last_cmd = "RTL"

    def get_altitude(self) -> Optional[float]:
        return self._altitude

    def close(self):
        logging.info("MockDrone: Session ended.")

    def get_last_command_str(self) -> str:
        if self._last_cmd == "VELOCITY":
            return f"vx={self._last_vx:+.2f} vy={self._last_vy:+.2f} vz={self._last_vz:+.2f}"
        return self._last_cmd
