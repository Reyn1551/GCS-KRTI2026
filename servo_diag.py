#!/usr/bin/env python3
"""
KP2026 — Diagnostik & Kalibrasi Servo PCA9685 (payload release ch0 & ch1).

Dipakai untuk isolasi masalah "perintah diterima tapi servo tidak bergerak":

  1. --scan     : cek PCA9685 terdeteksi di bus I2C (alamat).
  2. --pulse    : set PWM langsung (us) ke sebuah channel — membuktikan
                  chip hidup & sinyal keluar tanpa konversi sudut.
  3. --angle    : set posisi servo pada sudut tertentu (0..180).
  4. --sweep    : sweep halus 0 -> 180 -> 0 agar gerak servo mudah diamati.

Wiring checklist (cek fisik sebelum menyalahkan kode):
  - VCC+GND modul PCA9685  -> supply eksternal 5-6V yang cukup (BUKAN pin 3V3).
  - GND supply servo       -> GND Pi / GND PCA9685 harus SAMA (common ground).
  - Pin OE (enable) modul  -> harus di-ground (beberapa modul ada jumper).
  - Kabel sinyal servo     -> channel 0 dan/atau channel 1 PCA9685.
  - Servo butuh arus puncak 1-2A; supply via pin 5V Pi tidak selalu cukup.

Mode:
  python3 servo_diag.py --scan
  python3 servo_diag.py --pulse 1500 --channel 0
  python3 servo_diag.py --angle 90 --channel 0
  python3 servo_diag.py --sweep --both
  python3 servo_diag.py --dry-run           # tanpa sentuh hardware
"""

import argparse
import sys
import time
from typing import List, Optional, Tuple


def _import_hw() -> Optional[Tuple]:
    """Import library hardware saat dipakai (agar --dry-run jalan tanpa Pi)."""
    try:
        import board  # type: ignore[import-not-found]
        import busio  # type: ignore[import-not-found]
        from adafruit_pca9685 import PCA9685  # type: ignore[import-not-found]
        from adafruit_motor import servo as servo_lib  # type: ignore[import-not-found]

        return (board, busio, PCA9685, servo_lib)
    except (ImportError, ModuleNotFoundError) as e:
        print(f"[ERROR] Library hardware tidak tersedia: {e}", file=sys.stderr)
        print(
            "  Install: pip install adafruit-circuitpython-busdevice "
            "adafruit-circuitpython-motor adafruit-circuitpython-pca9685",
            file=sys.stderr,
        )
        return None


def _open_i2c(board, busio):
    """Buka bus I2C. Pi 5 default = /dev/i2c-1 (SDA pin3, SCL pin5)."""
    try:
        return busio.I2C(board.SCL, board.SDA)
    except Exception as e:  # noqa: BLE001
        print(f"[ERROR] Gagal buka I2C: {e}", file=sys.stderr)
        print(
            "  - Pastikan I2C aktif: sudo raspi-config > Interface Options > I2C\n"
            "  - Pastikan user punya akses: sudo usermod -a -G i2c $USER (lalu re-login)",
            file=sys.stderr,
        )
        return None


def find_pca(i2c) -> Optional[int]:
    """Scan bus I2C, cari alamat PCA9685 (urutan prioritas)."""
    addrs = i2c.scan()
    if not addrs:
        return None
    preferred = [0x40, 0x60, 0x70, 0x41]
    for a in preferred:
        if a in addrs:
            return a
    print(
        f"[WARN] Tidak ada alamat PCA9685 ({[hex(a) for a in preferred]}). "
        f"Terdeteksi: {[hex(a) for a in addrs]}"
    )
    return None


def _duty_from_us(usec: float) -> int:
    """Konversi lebar pulse (us) ke duty_cycle 16-bit versi library adafruit.

    Channel.duty_cycle library adafruit_pca9685 memakai skala 0..65535
    (16-bit), DI-INTERNAL-kan ke 12-bit oleh library. Jadi nilai harus
    dalam skala 65535, BUKAN 4095, akibatnya PWM keluar mendekati 0.
    """
    return int(usec / 20000.0 * 65535)


def pulse_test(channel, pca, usec: int, duration: float):
    """PWM mentah langsung ke channel, tahan `duration` detik, lalu lepas."""
    duty = _duty_from_us(usec)
    print(f"  Pulse {usec} us ke channel {channel} (duty_cycle={duty}) "
          f"{duration:.1f} s ...", flush=True)
    pca.channels[channel].duty_cycle = duty
    time.sleep(duration)
    pca.channels[channel].duty_cycle = 0
    print(f"  Channel {channel} dilepas (duty=0).")


def angle_set(servo, pca, channel: int, angle: float, delay: float = 0.0):
    pulse = 500 + angle / 180.0 * 2000.0
    print(f"  Channel {channel} -> sudut {angle:0.1f} deg "
          f"(pulsa ~{pulse:.0f} us)", flush=True)
    servo.angle = angle
    time.sleep(delay)
    pca.channels[channel].duty_cycle = 0


def sweep(servo, pca, channel: int, delay: float = 0.35):
    print(f"  Sweep channel {channel}: 0 -> 180 -> 0 deg")
    angles = list(range(0, 181, 10))
    angles += list(reversed(angles))
    try:
        for a in angles:
            servo.angle = a
            print(f"    angle={a:>3} deg", end="\r", flush=True)
            time.sleep(delay)
    finally:
        print()
        pca.channels[channel].duty_cycle = 0
        print(f"  Channel {channel} selesai (duty=0).")


def main():
    parser = argparse.ArgumentParser(
        description="KP2026 Servo PCA9685 — diagnostic & calibration tool",
        epilog="Contoh: %(prog)s --scan | %(prog)s --pulse 1500 --channel 0 "
               "| %(prog)s --sweep --both",
    )
    parser.add_argument("--scan", action="store_true", help="Deteksi PCA9685 di bus I2C")
    parser.add_argument("--channel", type=int, default=0, help="Index channel (0 atau 1)")
    parser.add_argument("--both", action="store_true", help="Aplikasi ke channel 0 & 1")
    parser.add_argument("--pulse", type=int,
                        help="Kirim pulse PWM mentah (us, mis. 1500), lalu lepas")
    parser.add_argument("--angle", type=float,
                        help="Set posisi servo (deg, 0..180)")
    parser.add_argument("--sweep", action="store_true", help="Sweep 0->180->0")
    parser.add_argument("--delay", type=float, default=0.35, help="Delay antar langkah (s)")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Durasi hold pulse saat --pulse (s)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Cetak rencana tanpa hardware (aman utk dev)")
    args = parser.parse_args()

    print("== KP2026 Servo PCA9685 Diagnostics ==")

    if args.dry_run:
        print("[DRY-RUN] Tanpa akses bus I2C. Rencana:")
        print("  - Scan I2C, cari PCA9685 (prioritas alamat 0x40)")
        target = "both (0&1)" if args.both else str(args.channel)
        print(f"  - Channel target: {target}")
        if args.pulse:
            print(f"  - Pulse {args.pulse} us selama {args.duration} s")
        if args.angle is not None:
            print(f"  - Set sudut {args.angle} deg")
        if args.sweep:
            print(f"  - Sweep 0->180->0 (delay={args.delay} s)")
        print("\n  Checklist firmware kalau saat ini tidak bergerak:")
        print("    1. Power PCA9685 5-6V eksternal + common ground servo/tegangan")
        print("    2. Pin OE (enable) modul di-ground")
        print("    3. Servo terpasang di channel yang benar (0/1)")
        return

    hw = _import_hw()
    if hw is None:
        sys.exit(1)
    board, busio, PCA9685, mservo = hw

    i2c = _open_i2c(board, busio)
    if i2c is None:
        sys.exit(1)

    addr = find_pca(i2c)
    if addr is None:
        present = [hex(a) for a in i2c.scan()] if i2c.scan() else []
        print("\n[FAIL] PCA9685 tidak terdeteksi di bus I2C.")
        print("  Periksa:")
        print("   - Supply power modul (VCC/GND) aktif?")
        print("   - SDA (pin3) dan SCL (pin5) tersambung dengan benar?")
        print("   - Alamat I2C berbeda? cek: sudo i2cdetect -y 1")
        print(f"   Perangkat terdeteksi: {present}")
        sys.exit(2)

    print(f"[OK] PCA9685 terdeteksi di 0x{addr:02X}.")
    try:
        pca = PCA9685(i2c, address=addr)
        pca.frequency = 50  # 50 Hz standar servo
        print("  Frekuensi PWM = 50 Hz di-set.")
    except Exception as e:  # noqa: BLE001
        print(f"[ERROR] Gagal inisialisasi PCA9685: {e}", file=sys.stderr)
        sys.exit(3)

    servo_map = {
        c: mservo.Servo(pca.channels[c], min_pulse=500, max_pulse=2500)
        for c in range(4)
    }
    channels = [0, 1, 2, 3] if args.both else [args.channel]

    if args.scan:
        print("[OK] Deteksi selesai — tidak ada aksi servo.")
        return

    for c in channels:
        if args.pulse:
            pulse_test(c, pca, args.pulse, args.duration)
        elif args.angle is not None:
            angle_set(servo_map[c], pca, c, args.angle, delay=args.delay)
        elif args.sweep:
            sweep(servo_map[c], pca, c, delay=args.delay)
        else:
            print(f"[WARN] Channel {c}: belum ada aksi. Gunakan --pulse/--angle/--sweep.")

    for i in range(4):
        pca.channels[i].duty_cycle = 0
    print("[OK] Semua channel 0-3 dilepas (duty_cycle=0). Selesai.")


if __name__ == "__main__":
    main()