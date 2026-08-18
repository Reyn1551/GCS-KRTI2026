#!/usr/bin/env python3
"""
KP2026 — MAVLink Mission Planner Servo Toggle.

Enables toggling the payload release servo directly from Mission Planner
via MAV_CMD_DO_SET_RELAY commands. Also supports the legacy SERVO_OUTPUT_RAW
method from the flight controller.

How to use from Mission Planner:
  1. Connect Mission Planner to the same MAVLink network
  2. Go to Actions tab or MAVLink Inspector
  3. Send COMMAND_LONG: MAV_CMD_DO_SET_RELAY (181)
     - Param1 (Relay#):  1
     - Param2 (On/Off):  1 = BUKA (drop), 0 = TUTUP (secure)
  4. Or use the simpler SERVO_OUTPUT_RAW method via FC servo output

Usage:
    python3 control/mavlink_toggle_servo.py --port /dev/ttyAMA0
    python3 control/mavlink_toggle_servo.py --sim
"""

import argparse
import logging
import os
import signal
import sys
import time
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config as cfg
from control.servo import ServoController

try:
    from pymavlink import mavutil  # type: ignore[import-untyped]

    HAS_MAVLINK = True
except ImportError:
    mavutil = None  # type: ignore[assignment]
    HAS_MAVLINK = False

logger = logging.getLogger("mavlink_toggle")

# Custom component ID so Mission Planner sees us as a separate device
COMPANION_COMP_ID = 191  # MAV_COMP_ID_USER14 (free for custom use)

# Relay number used for servo toggle
SERVO_RELAY_NUM = 1


class MavlinkServoToggle:
    """Listens for MAVLink relay commands and toggles physical servo."""

    def __init__(
        self,
        port: str = cfg.SERIAL_PORT,
        baud: int = cfg.SERIAL_BAUD,
        host: Optional[str] = None,
        dry_run: bool = False,
        debug: bool = False,
    ):
        if not HAS_MAVLINK:
            raise RuntimeError(
                "pymavlink not installed. Use --sim for testing without MAVLink."
            )

        self.port = port
        self.baud = baud
        self.host = host
        self.dry_run = dry_run
        self.debug = debug

        if host:
            self._connection_string = f"udp:{host}"
        else:
            self._connection_string = port
        self.servo = ServoController(dry_run=dry_run)
        self.master: Optional[mavutil.mavfile] = None
        self.target_system = 1
        self.target_component = 1
        self._running = False
        self._command_count = 0
        self._error_count = 0
        self._last_heartbeat = 0.0
        self._heartbeat_timeout = False
        self._prev_servo10_pwm: Optional[int] = None

    def connect(self, retries: int = 3, retry_delay: float = 2.0) -> bool:
        logger.info("Connecting to %s...", self._connection_string)
        for attempt in range(retries):
            try:
                self.master = mavutil.mavlink_connection(
                    device=self._connection_string,
                    baud=self.baud,
                    source_system=255,
                    source_component=COMPANION_COMP_ID,
                )
                msg = self.master.wait_heartbeat(timeout=10)
                if msg is None:
                    raise ConnectionError("No heartbeat from flight controller")
                self.target_system = msg.get_srcSystem()
                self.target_component = msg.get_srcComponent()
                self._last_heartbeat = time.monotonic()
                self._heartbeat_timeout = False
                logger.info(
                    "Connected — FC: sys=%d comp=%d  Ours: comp=%d",
                    self.target_system, self.target_component, COMPANION_COMP_ID,
                )
                return True
            except Exception as e:
                logger.warning("Connection attempt %d/%d failed: %s", attempt + 1, retries, e)
                if self.master:
                    try:
                        self.master.close()
                    except Exception:
                        pass
                    self.master = None
                if attempt < retries - 1:
                    time.sleep(retry_delay)
        logger.error("All %d connection attempts failed.", retries)
        return False

    def disconnect(self):
        self._running = False
        self.servo.lepas()
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None
        logger.info("Disconnected.")

    # ── MAVLink Message Senders ──────────────────────────────────────────

    def _send_statustext(self, text: str, severity: int = 6):
        """Send STATUSTEXT to appear in Mission Planner's Messages tab."""
        if not self.master:
            return
        self.master.mav.statustext_send(severity, text.encode("utf-8", errors="replace"))
        if self.debug:
            logger.debug("STATUSTEXT: [%d] %s", severity, text)

    def _send_command_ack(self, command: int, result: int = 0):
        """Acknowledge a command (result: 0=MAV_RESULT_ACCEPTED)."""
        if not self.master:
            return
        self.master.mav.command_ack_send(
            command,
            result,
            0, 0,
            self.target_system,
            COMPANION_COMP_ID,
        )

    def _send_heartbeat(self):
        """Send heartbeat so Mission Planner detects our component."""
        if not self.master:
            return
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )

    # ── Command Handlers ─────────────────────────────────────────────────

    def _handle_command_long(self, msg):
        """Handle COMMAND_LONG — primary toggle mechanism from Mission Planner."""
        cmd = msg.command
        param1 = msg.param1
        param2 = msg.param2
        confirmation = int(msg.param7)  # 1 = need confirmation, 0 = execute

        if cmd == mavutil.mavlink.MAV_CMD_DO_SET_RELAY:
            relay_num = int(param1)
            relay_state = int(param2)

            if relay_num != SERVO_RELAY_NUM:
                return  # Not our relay

            if confirmation == 1:
                self._send_command_ack(cmd, 0)  # Accept
                return

            if relay_state == 1:
                self._servo_buka(f"Mission Planner (relay {SERVO_RELAY_NUM})")
            else:
                self._servo_tutup(f"Mission Planner (relay {SERVO_RELAY_NUM})")

            self._send_command_ack(cmd, 0)

        elif cmd == mavutil.mavlink.MAV_CMD_USER_1:
            self._servo_buka("Mission Planner (USER_1)")
            self._send_command_ack(cmd, 0)

        elif cmd == mavutil.mavlink.MAV_CMD_USER_2:
            self._servo_tutup("Mission Planner (USER_2)")
            self._send_command_ack(cmd, 0)

    def _handle_servo_output_raw(self, msg):
        """Legacy method: read servo output from flight controller."""
        servo10_pwm = getattr(msg, "servo10_raw", -1)
        if servo10_pwm < 0:
            return

        if self._prev_servo10_pwm is not None and servo10_pwm != self._prev_servo10_pwm:
            if servo10_pwm == cfg.PWM_BUKA:
                self._servo_buka(f"SERVO_OUTPUT_RAW (PWM {servo10_pwm})")
            elif servo10_pwm == cfg.PWM_TUTUP:
                self._servo_tutup(f"SERVO_OUTPUT_RAW (PWM {servo10_pwm})")

        self._prev_servo10_pwm = servo10_pwm

    # ── Servo Actions ────────────────────────────────────────────────────

    def _servo_buka(self, source: str):
        self.servo.buka()
        self._command_count += 1
        text = f"Servo BUKA — {source}"
        logger.info(text)
        self._send_statustext(text, severity=4)  # MAV_SEVERITY_WARNING (orange)

    def _servo_tutup(self, source: str):
        self.servo.tutup()
        self._command_count += 1
        text = f"Servo TUTUP — {source}"
        logger.info(text)
        self._send_statustext(text, severity=6)  # MAV_SEVERITY_INFO

    # ── Safety ───────────────────────────────────────────────────────────

    def _check_heartbeat_timeout(self):
        elapsed = time.monotonic() - self._last_heartbeat
        if elapsed > cfg.HEARTBEAT_TIMEOUT_S and not self._heartbeat_timeout:
            logger.warning(
                "HEARTBEAT TIMEOUT (%.1fs) — servo released (fail-safe)", elapsed,
            )
            self.servo.lepas()
            self._send_statustext(
                f"Servo FAILSAFE: released after {elapsed:.0f}s heartbeat timeout",
                severity=2,  # MAV_SEVERITY_CRITICAL
            )
            self._heartbeat_timeout = True

    # ── Main Loop ────────────────────────────────────────────────────────

    def run(self, wait: bool = False, retries: int = 3):
        if wait:
            while not self.connect(retries=999999, retry_delay=3.0):
                logger.warning("FC not available. Waiting 5s before retry...")
                time.sleep(5.0)
        elif not self.connect(retries=retries, retry_delay=2.0):
            logger.error("Cannot connect to %s.", self._connection_string)
            logger.error("Possible causes:")
            if self.host:
                logger.error("  - No MAVLink router at %s (run: mavproxy --master=/dev/ttyAMA0 --out=udp:127.0.0.1:14550)", self.host)
            else:
                logger.error("  - FC not powered / not connected to %s", self.port)
                logger.error("  - Wrong baud rate (config: %d)", self.baud)
                logger.error("  - Another process using the port (lsof %s)", self.port)
            logger.error("  - Run with --sim for testing without hardware")
            return

        self._running = True
        self._send_heartbeat()

        logger.info("=" * 55)
        logger.info("Listening for commands from Mission Planner...")
        logger.info("  Relay %d: ON/OFF for servo BUKA/TUTUP", SERVO_RELAY_NUM)
        logger.info("  USER_1: BUKA   USER_2: TUTUP")
        logger.info("  SERVO_OUTPUT_RAW servo %d: legacy method", cfg.SERVO_NO_TARGET)
        logger.info("  Heartbeat timeout: %.0fs (fail-safe release)", cfg.HEARTBEAT_TIMEOUT_S)
        logger.info("=" * 55)

        last_heartbeat_send = 0.0

        while self._running:
            try:
                msg = self.master.recv_match(blocking=True, timeout=0.1)
                t_now = time.monotonic()

                if msg is not None:
                    msg_type = msg.get_type()

                    if msg_type == "HEARTBEAT":
                        self._last_heartbeat = t_now
                        self._heartbeat_timeout = False

                    elif msg_type == "COMMAND_LONG":
                        if msg.target_component in (0, COMPANION_COMP_ID):
                            self._handle_command_long(msg)

                    elif msg_type == "SERVO_OUTPUT_RAW":
                        self._handle_servo_output_raw(msg)

                # Periodic heartbeat send (1 Hz) — so Mission Planner sees us
                if t_now - last_heartbeat_send > 1.0:
                    self._send_heartbeat()
                    last_heartbeat_send = t_now

                self._check_heartbeat_timeout()

            except KeyboardInterrupt:
                logger.info("KeyboardInterrupt received.")
                break
            except Exception as e:
                self._error_count += 1
                logger.error("Error in main loop: %s", e)
                if self._error_count > 20:
                    logger.critical("Too many errors, exiting.")
                    break
                time.sleep(0.5)

        self.disconnect()
        self._print_summary()

    def _print_summary(self):
        print("\n" + "=" * 55)
        print("  MAVLink Servo Toggle — Selesai")
        print("=" * 55)
        print(f"  Commands executed: {self._command_count}")
        print(f"  Errors: {self._error_count}")
        print(f"  Servo state: {'BUKA' if self.servo.is_open else 'TUTUP'}")
        print("=" * 55)


# ═══════════════════════════════════════════════════════════════════════════
# Simulation
# ═══════════════════════════════════════════════════════════════════════════


def simulate_toggle():
    """Simulate servo toggle locally without MAVLink hardware."""
    print("\nSIMULATION — MAVLink Servo Toggle\n")
    servo = ServoController(dry_run=True)

    print("  → Simulating relay ON (BUKA):")
    servo.buka()
    time.sleep(0.3)

    print("  → Simulating relay OFF (TUTUP):")
    servo.tutup()
    time.sleep(0.3)

    print("\n  Servo state:", "BUKA" if servo.is_open else "TUTUP")
    servo.lepas()
    print("  Servo released.\n")


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════


def setup_logging():
    os.makedirs(cfg.LOG_DIR, exist_ok=True)
    log_file = os.path.join(
        cfg.LOG_DIR, f"mavlink_toggle_{time.strftime('%Y%m%d_%H%M%S')}.log"
    )
    formatter = logging.Formatter(
        fmt="%(asctime)s.%(msecs)03d [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    fh = logging.FileHandler(log_file)
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(formatter)
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.INFO)
    ch.setFormatter(formatter)
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    root.addHandler(fh)
    root.addHandler(ch)
    logger.info("Log file: %s", log_file)


def main():
    parser = argparse.ArgumentParser(
        description="KP2026 MAVLink Servo Toggle — Mission Planner integration"
    )
    parser.add_argument("--port", default=cfg.SERIAL_PORT, help="MAVLink serial port")
    parser.add_argument("--baud", type=int, default=cfg.SERIAL_BAUD, help="Baud rate")
    parser.add_argument("--host", default=cfg.MAVLINK_HOST,
                        help=f"UDP connection instead of serial (default: {cfg.MAVLINK_HOST})")
    parser.add_argument("--dry-run", action="store_true", help="Log actions without hardware")
    parser.add_argument("--sim", action="store_true", help="Run local simulation")
    parser.add_argument("--debug", action="store_true", help="Verbose MAVLink diagnostics")
    parser.add_argument("--wait", action="store_true", help="Keep retrying until FC connects")
    parser.add_argument("--retries", type=int, default=3, help="Connection retries (default: 3)")
    args = parser.parse_args()

    setup_logging()

    conn_info = f"udp:{args.host}" if args.host else f"{args.port}@{args.baud}"
    print(f"""
    ╔══════════════════════════════════════════════════════╗
    ║  KP2026 MAVLink Servo Toggle — Mission Planner       ║
    ║  Payload Release (2 Servos: PCA9685 Ch 3-4)          ║
    ║  via MAV_CMD_DO_SET_RELAY                            ║
    ╚══════════════════════════════════════════════════════╝

    Connection: {conn_info}
    Commands accepted from Mission Planner:
      MAV_CMD_DO_SET_RELAY {SERVO_RELAY_NUM}=ON  → BUKA 2 Servos (PWM {cfg.PWM_BUKA})
      MAV_CMD_DO_SET_RELAY {SERVO_RELAY_NUM}=OFF → TUTUP 2 Servos (PWM {cfg.PWM_TUTUP})
      MAV_CMD_USER_1 (31000)                     → BUKA 2 Servos
      MAV_CMD_USER_2 (31001)                     → TUTUP 2 Servos

    Legacy: SERVO_OUTPUT_RAW servo {cfg.SERVO_NO_TARGET} (PWM {cfg.PWM_BUKA}/{cfg.PWM_TUTUP})
    Heartbeat timeout: {cfg.HEARTBEAT_TIMEOUT_S:.0f}s → auto-release (fail-safe)
    """)

    if args.sim:
        simulate_toggle()
        return

    if not HAS_MAVLINK:
        logger.error("pymavlink not installed. Run with --sim.")
        sys.exit(1)

    toggle = MavlinkServoToggle(
        port=args.port, baud=args.baud, host=args.host,
        dry_run=args.dry_run, debug=args.debug,
    )

    def signal_handler(signum, frame):
        logger.info("Signal %d — shutting down...", signum)
        toggle.disconnect()
        sys.exit(0)

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        toggle.run(wait=args.wait, retries=args.retries)
    except Exception as e:
        logger.critical("Fatal: %s", e)
        toggle.disconnect()
        sys.exit(1)


if __name__ == "__main__":
    main()
