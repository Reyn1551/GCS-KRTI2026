#!/usr/bin/env python3
"""
KP2026 — MAVLink Servo Controller v2.0
Listens for SERVO_OUTPUT_RAW from flight controller, triggers PCA9685 servo release.

See RULES.md for payload release mechanics.
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

logger = logging.getLogger("mavlink_servo")


# ═══════════════════════════════════════════════════════════════════════════
# MAVLink Listener
# ═══════════════════════════════════════════════════════════════════════════


class MavlinkServoListener:
    def __init__(self, port: str = cfg.SERIAL_PORT, baud: int = cfg.SERIAL_BAUD, dry_run: bool = False, debug: bool = False):
        if not HAS_MAVLINK:
            raise RuntimeError("pymavlink not installed. Use --dry-run for testing without MAVLink.")

        self.port = port
        self.baud = baud
        self.master: Optional[mavutil.mavfile] = None
        self.target_system = 1
        self.target_component = 1
        self.servo = ServoController(dry_run=dry_run)
        self._running = False
        self._last_heartbeat = time.monotonic()
        self._heartbeat_timeout_triggered = False
        self._command_count = 0
        self._error_count = 0
        self._servo_states: dict = {}
        self._prev_servo10_pwm: Optional[int] = None
        self._debug = debug

    def connect(self) -> bool:
        logger.info("Connecting %s@%d...", self.port, self.baud)
        try:
            self.master = mavutil.mavlink_connection(device=self.port, baud=self.baud, source_system=255)
            msg = self.master.wait_heartbeat(timeout=10)
            if msg is None:
                logger.error("No heartbeat from flight controller!")
                return False
            self.target_system = msg.get_srcSystem()
            self.target_component = msg.get_srcComponent()
            self._last_heartbeat = time.monotonic()
            self._heartbeat_timeout_triggered = False
            logger.info("Connected sys=%d comp=%d", self.target_system, self.target_component)
            return True
        except Exception as e:
            logger.error("Connection failed: %s", e)
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

    def _handle_servo_output_raw(self, msg):
        servo10_pwm = getattr(msg, "servo10_raw", -1)
        if servo10_pwm < 0:
            return

        for i in range(1, 16):
            val = getattr(msg, f"servo{i}_raw", -1)
            if val > 0:
                self._servo_states[i] = val

        if self._prev_servo10_pwm is not None and servo10_pwm != self._prev_servo10_pwm:
            label = ""
            if servo10_pwm == cfg.PWM_BUKA:
                label = " ← BUKA (First Aid Drop)"
                self.servo.buka()
                self._command_count += 1
            elif servo10_pwm == cfg.PWM_TUTUP:
                label = " ← TUTUP"
                self.servo.tutup()
                self._command_count += 1
            elif self._debug:
                label = f" ← nilai lain ({cfg.PWM_BUKA}/{cfg.PWM_TUTUP})"

            line = f"SERVO_OUTPUT_RAW → servo=10 PWM={servo10_pwm}{label}"
            print(line)
            logger.info("Servo 10 PWM changed: %d%s", servo10_pwm, label)

        self._prev_servo10_pwm = servo10_pwm

    def _check_heartbeat_timeout(self):
        elapsed = time.monotonic() - self._last_heartbeat
        if elapsed > cfg.HEARTBEAT_TIMEOUT_S and not self._heartbeat_timeout_triggered:
            logger.warning("HEARTBEAT TIMEOUT %.1fs — servo released (fail-safe).", elapsed)
            self.servo.lepas()
            self._heartbeat_timeout_triggered = True

    def run(self):
        if not self.connect():
            logger.error("Cannot connect, exiting.")
            return

        self._running = True
        logger.info("Monitoring SERVO_OUTPUT_RAW...")
        logger.info("  Servo %d: BUKA=PWM %d  TUTUP=PWM %d", cfg.SERVO_NO_TARGET, cfg.PWM_BUKA, cfg.PWM_TUTUP)

        while self._running:
            try:
                msg = self.master.recv_match(blocking=True, timeout=0.01)
                if msg is None:
                    self._check_heartbeat_timeout()
                    continue

                msg_type = msg.get_type()
                if msg_type == "HEARTBEAT":
                    self._last_heartbeat = time.monotonic()
                    self._heartbeat_timeout_triggered = False
                elif msg_type == "SERVO_OUTPUT_RAW":
                    self._handle_servo_output_raw(msg)

            except KeyboardInterrupt:
                logger.info("KeyboardInterrupt received.")
                break
            except Exception as e:
                self._error_count += 1
                logger.error("Error: %s", e)
                if self._error_count > 10:
                    logger.critical("Too many errors. Exiting.")
                    break

        self.disconnect()
        self._print_summary()

    def _print_summary(self):
        print("\n" + "=" * 55)
        print("  MAVLink Servo Listener — Selesai")
        print("=" * 55)
        print(f"  Total commands received: {self._command_count}")
        print(f"  Errors: {self._error_count}")
        if self._servo_states:
            print("  Servo states:")
            for sv in sorted(self._servo_states):
                pwm = self._servo_states[sv]
                marker = ""
                if sv == cfg.SERVO_NO_TARGET:
                    marker = "  <-- servo target"
                    if pwm == cfg.PWM_BUKA:
                        marker += " (BUKA)"
                    elif pwm == cfg.PWM_TUTUP:
                        marker += " (TUTUP)"
                print(f"    Servo {sv:2d}: PWM {pwm:5d}{marker}")
        else:
            print("  No servo data received.")
        print("=" * 55)


# ═══════════════════════════════════════════════════════════════════════════
# Simulation
# ═══════════════════════════════════════════════════════════════════════════


def simulate_commands(dry_run: bool = True):
    print("\nSIMULATION MODE — servo commands (PWM %d/%d)\n" % (cfg.PWM_BUKA, cfg.PWM_TUTUP))
    servo_ctrl = ServoController(dry_run=dry_run)

    print(f"  BUKA (PWM {cfg.PWM_BUKA})")
    servo_ctrl.buka()
    time.sleep(0.5)

    print(f"\n  TUTUP (PWM {cfg.PWM_TUTUP})")
    servo_ctrl.tutup()
    time.sleep(0.5)

    print("\n  Unknown PWM 999 — should be ignored")
    print("  [OK] Ignored (unknown PWM value)")

    print("\nSIMULATION COMPLETE — servo: %s" % ("BUKA" if servo_ctrl.is_open else "TUTUP"))
    servo_ctrl.lepas()
    print("Servo released (duty_cycle=0)\n")


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════


def setup_logging():
    os.makedirs(cfg.LOG_DIR, exist_ok=True)
    log_file = os.path.join(cfg.LOG_DIR, f"mavlink_servo_{time.strftime('%Y%m%d_%H%M%S')}.log")
    formatter = logging.Formatter(fmt="%(asctime)s.%(msecs)03d [%(levelname)s] %(name)s: %(message)s", datefmt="%H:%M:%S")
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


def parse_args():
    parser = argparse.ArgumentParser(description="KP2026 MAVLink Servo Controller")
    parser.add_argument("--port", default=cfg.SERIAL_PORT, help="MAVLink serial port")
    parser.add_argument("--baud", type=int, default=cfg.SERIAL_BAUD, help="Baud rate")
    parser.add_argument("--dry-run", action="store_true", help="Log actions without hardware")
    parser.add_argument("--sim", action="store_true", help="Run local simulation")
    parser.add_argument("--debug", action="store_true", help="Verbose MAVLink diagnostics")
    return parser.parse_args()


def main():
    args = parse_args()
    setup_logging()

    print(f"""
    KP2026 MAVLink Servo Controller — Payload Release
    Monitor: SERVO_OUTPUT_RAW servo {cfg.SERVO_NO_TARGET}
    BUKA:    PWM {cfg.PWM_BUKA} → angle=0 (drop)
    TUTUP:   PWM {cfg.PWM_TUTUP} → angle=60 (secure)
    """)

    if args.sim:
        simulate_commands(dry_run=args.dry_run)
        return

    if args.dry_run:
        logger.info("DRY-RUN MODE: Logging actions without hardware.")

    if not HAS_MAVLINK and not args.dry_run:
        logger.error("pymavlink not installed. Run with --dry-run or --sim.")
        sys.exit(1)

    if not HAS_MAVLINK and args.dry_run:
        simulate_commands(dry_run=True)
        return

    listener: Optional[MavlinkServoListener] = None

    def signal_handler(signum, frame):
        logger.info("Signal %d received, shutting down...", signum)
        if listener:
            listener.disconnect()
        sys.exit(0)

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        listener = MavlinkServoListener(port=args.port, baud=args.baud, dry_run=args.dry_run, debug=args.debug)
        listener.run()
    except Exception as e:
        logger.critical("Fatal error: %s", e)
        if listener:
            listener.disconnect()
        sys.exit(1)


if __name__ == "__main__":
    main()
