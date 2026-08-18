#!/usr/bin/env python3
"""
KP2026 — PCA9685 Servo Controller for Payload Release.

Controls channel 3 and 4 on PCA9685 module for First Aid Kit release.
Modes: normal (hardware), dry-run (print only), importable as library.

Usage:
  python3 control/servo.py          # Interactive mode (keyboard)
  python3 control/servo.py --dry-run # Test without hardware
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import PWM_BUKA, PWM_TUTUP

# PCA9685 channels used for payload release servos
SERVO_CHANNELS = (3, 4)

# Lazy imports — only when actually using hardware
_board = None
_busio = None
_adafruit_pca = None
_adafruit_servo = None


def _init_hardware():
    """Import hardware libraries (only works on Raspberry Pi)."""
    global _board, _busio, _adafruit_pca, _adafruit_servo
    if _board is not None:
        return True
    try:
        import board as _b
        import busio as _bi
        from adafruit_pca9685 import PCA9685
        from adafruit_motor import servo as servo_lib

        _board = _b
        _busio = _bi
        _adafruit_pca = PCA9685
        _adafruit_servo = servo_lib
        return True
    except ImportError as e:
        print(f"Cannot import hardware libraries: {e}", file=sys.stderr)
        print("Install with: pip install adafruit-circuitpython-pca9685 adafruit-circuitpython-motor", file=sys.stderr)
        return False


class ServoController:
    """Wrapper for PCA9685 servo (ch3 & ch4) with dry-run support."""

    def __init__(self, dry_run: bool = False):
        self.dry_run = dry_run
        self._initialized = False
        self._servo_buka = False
        self._servos = []
        self._pca = None

        if not dry_run:
            self._init_hardware_device()

    def _init_hardware_device(self):
        if not _init_hardware():
            print("Falling back to dry-run mode.", file=sys.stderr)
            self.dry_run = True
            return
        try:
            i2c = _busio.I2C(_board.SCL, _board.SDA)
            self._pca = _adafruit_pca(i2c)
            self._pca.frequency = 50
            self._servos = [
                _adafruit_servo.Servo(
                    self._pca.channels[i], min_pulse=500, max_pulse=2500
                )
                for i in SERVO_CHANNELS
            ]
            self._initialized = True
        except Exception as e:
            print(f"Hardware init failed: {e}", file=sys.stderr)
            print("Falling back to dry-run mode.", file=sys.stderr)
            self.dry_run = True

    def buka(self):
        """Open servo — drop First Aid Kit."""
        self._servo_buka = True
        if self.dry_run:
            print(f"[DRY-RUN] Servo: BUKA (PWM {PWM_BUKA})")
            return
        if not self._initialized:
            print("[WARN] Hardware not initialized, skipping BUKA", file=sys.stderr)
            return
        try:
            for s in self._servos:
                s.angle = 0
            print(f"Servo (Channels 3-4): BUKA (angle=0, PWM {PWM_BUKA})")
        except Exception as e:
            print(f"Servo BUKA failed: {e}", file=sys.stderr)

    def tutup(self):
        """Close servo — secure payload."""
        self._servo_buka = False
        if self.dry_run:
            print(f"[DRY-RUN] Servo: TUTUP (PWM {PWM_TUTUP})")
            return
        if not self._initialized:
            print("[WARN] Hardware not initialized, skipping TUTUP", file=sys.stderr)
            return
        try:
            for s in self._servos:
                s.angle = 60
            print(f"Servo (Channels 3-4): TUTUP (angle=60, PWM {PWM_TUTUP})")
        except Exception as e:
            print(f"Servo TUTUP failed: {e}", file=sys.stderr)

    def lepas(self):
        """Release PWM signal — servo stops holding position."""
        if self.dry_run:
            print("[DRY-RUN] Servo: LEPAS (duty_cycle=0)")
            return
        if not self._initialized:
            return
        try:
            for i in SERVO_CHANNELS:
                self._pca.channels[i].duty_cycle = 0
            print("Servo (Channels 3-4): LEPAS (duty_cycle=0)")
        except Exception as e:
            print(f"Servo LEPAS failed: {e}", file=sys.stderr)

    @property
    def is_open(self) -> bool:
        return self._servo_buka


# ═══════════════════════════════════════════════════════════════════════════
# Interactive keyboard control
# ═══════════════════════════════════════════════════════════════════════════


def _read_char() -> str:
    """Read single char without Enter key (Unix only)."""
    try:
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch
    except ImportError:
        return sys.stdin.read(1)


def interactive_main(dry_run: bool = False):
    servo = ServoController(dry_run=dry_run)

    print("KP2026 Servo Controller (PCA9685 Channels: 3 & 4)")
    print(f"  o = buka (PWM {PWM_BUKA})")
    print(f"  c = tutup (PWM {PWM_TUTUP})")
    print("  q = keluar")
    print("-" * 40)

    try:
        while True:
            ch = _read_char().lower()
            if ch == "o":
                servo.buka()
            elif ch == "c":
                servo.tutup()
            elif ch == "q":
                print("Exiting...")
                break
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        servo.lepas()


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════


def main():
    parser = argparse.ArgumentParser(description="KP2026 PCA9685 Servo Controller")
    parser.add_argument("--dry-run", action="store_true", help="Test without hardware")
    parser.add_argument("--buka", action="store_true", help="Open servo and exit")
    parser.add_argument("--tutup", action="store_true", help="Close servo and exit")
    args = parser.parse_args()

    if args.buka:
        ServoController(dry_run=args.dry_run).buka()
    elif args.tutup:
        ServoController(dry_run=args.dry_run).tutup()
    else:
        interactive_main(dry_run=args.dry_run)


if __name__ == "__main__":
    main()
