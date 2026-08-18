"""
pca_servo.py - MAVProxy module: PCA9685 payload-release servo group (KP2026).

Mirrors the flight controller's servo output channel 10 onto TWO PCA9685
servos (channels 0 and 1) that always move together as one group.

How it works
------------
The GCS (a manual OPEN/CLOSE button, or a DO_SET_SERVO mission item in AUTO)
sends MAV_CMD_DO_SET_SERVO for servo 10 with PWM 1900 (BUKA) or 1100 (TUTUP).
The flight controller executes it, and the new value shows up in the
SERVO_OUTPUT_RAW telemetry stream. This module watches that stream and drives
both PCA9685 channels to match:

    FC servo 10 = 1900 (PWM_BUKA)  -> PCA ch0 + ch1, angle 0   (drop payload)
    FC servo 10 = 1100 (PWM_TUTUP) -> PCA ch0 + ch1, angle 60  (secure payload)

Why mirror SERVO_OUTPUT_RAW instead of intercepting DO_SET_SERVO?
  * one source of truth: whatever the FC outputs, the payload servo follows
  * works identically for AUTO mission items AND direct GCS commands
    (ArduPilot executes DO_SET_SERVO in any mode, including GUIDED)
  * FC failsafe outputs propagate automatically

Fail-safe: if the FC heartbeat is lost for HEARTBEAT_TIMEOUT_S seconds, the
PCA outputs are released (duty_cycle = 0) so the servos stop holding torque.

Install on the Pi:
    sudo pip3 install adafruit-circuitpython-pca9685 adafruit-circuitpython-motor
    mkdir -p ~/.mavproxy/modules
    cp pca_servo.py ~/.mavproxy/modules/     # or keep next to your launch script

Launch MAVProxy with:  --load-module pca_servo

MAVProxy console commands (bench test, no GCS/FC needed):
    pcaservo buka     # open  - both channels to angle 0
    pcaservo tutup    # close - both channels to angle 60
    pcaservo lepas    # release PWM (duty_cycle = 0)

Required ArduPilot params:
    SERVO10_FUNCTION = 0    (disabled - driven only by DO_SET_SERVO)
    servo output 10 must be a real PWM output (not GPIO/relay)
"""

import time

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Configuration (matches KP2026 config)
# ---------------------------------------------------------------------------

TRIGGER_SERVO = 10        # FC output channel the GCS/mission commands
PWM_BUKA = 1900           # FC PWM value meaning "open"  (drop payload)
PWM_TUTUP = 1100          # FC PWM value meaning "close" (secure payload)
PWM_TOLERANCE = 10        # us tolerance when matching trigger values

PCA_CHANNELS = (0, 1)     # both PCA9685 channels move as one group
ANGLE_BUKA = 0            # PCA-side angle for open
ANGLE_TUTUP = 60          # PCA-side angle for close
MIN_PULSE_US = 500        # adafruit_motor.Servo pulse range
MAX_PULSE_US = 2500
PCA_FREQ_HZ = 50
I2C_ADDRESS = 0x40

HEARTBEAT_TIMEOUT_S = 5.0
STREAM_REQUEST_HZ = 4     # requested SERVO_OUTPUT_RAW rate
STREAM_RETRY_S = 5.0

# ---------------------------------------------------------------------------

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor.servo import Servo
    HAVE_HW = True
except Exception:
    HAVE_HW = False


class PCAServoModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(PCAServoModule, self).__init__(
            mpstate, "pca_servo", "PCA9685 payload servo group", public=True
        )
        self.pca = None
        self.servos = []

        self._last_pwm = None
        self._seen_servo_output = False
        self._last_stream_request = 0.0

        self._seen_fc_heartbeat = False
        self._last_heartbeat = time.time()
        self._released = False

        if HAVE_HW:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.pca = PCA9685(i2c, address=I2C_ADDRESS)
                self.pca.frequency = PCA_FREQ_HZ
                self.servos = [
                    Servo(
                        self.pca.channels[ch],
                        min_pulse=MIN_PULSE_US,
                        max_pulse=MAX_PULSE_US,
                    )
                    for ch in PCA_CHANNELS
                ]
                print("pca_servo: PCA9685 ready at 0x%02X, channels %s"
                      % (I2C_ADDRESS, list(PCA_CHANNELS)))
            except Exception as exc:
                print("pca_servo: PCA9685 init failed: %s" % exc)
        else:
            print("pca_servo: hardware libs missing - logging only (dry-run)")

        self.add_command(
            "pcaservo", self.cmd_pcaservo,
            "payload servo group control", "<buka|tutup|lepas>",
        )

    # ------------------------------------------------------------------ #
    # servo group actions
    # ------------------------------------------------------------------ #

    def _set_angle(self, angle, label):
        self._released = False
        if not self.servos:
            print("pca_servo: [dry-run] %s (angle=%d)" % (label, angle))
            return
        try:
            for srv in self.servos:
                srv.angle = angle
            print("pca_servo: %s (angle=%d, ch %s)" % (label, angle, list(PCA_CHANNELS)))
        except Exception as exc:
            print("pca_servo: %s failed: %s" % (label, exc))

    def buka(self, auto_close_delay=None):
        """Open - drop the payload."""
        self._set_angle(ANGLE_BUKA, "BUKA")
        if auto_close_delay and auto_close_delay > 0:
            import threading
            def _auto_close():
                time.sleep(auto_close_delay)
                self.tutup()
            threading.Thread(target=_auto_close, daemon=True).start()

    def tutup(self):
        """Close - secure the payload."""
        self._set_angle(ANGLE_TUTUP, "TUTUP")

    def lepas(self):
        """Release the PWM signal - servo stops holding position."""
        self._released = True
        if self.pca is None:
            print("pca_servo: [dry-run] LEPAS (duty_cycle=0)")
            return
        try:
            for ch in PCA_CHANNELS:
                self.pca.channels[ch].duty_cycle = 0
            print("pca_servo: LEPAS (duty_cycle=0)")
        except Exception as exc:
            print("pca_servo: LEPAS failed: %s" % exc)

    # ------------------------------------------------------------------ #
    # MAVLink hook - sees traffic on every MAVProxy link (FC and GCS sides)
    # ------------------------------------------------------------------ #

    def mavlink_packet(self, m):
        mtype = m.get_type()

        if mtype == "HEARTBEAT":
            if m.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                self._seen_fc_heartbeat = True
                self._last_heartbeat = time.time()
            return

        if mtype != "SERVO_OUTPUT_RAW":
            return

        self._seen_servo_output = True
        pwm = getattr(m, "servo%d_raw" % TRIGGER_SERVO, None)
        if pwm is None or pwm == self._last_pwm:
            return

        self._last_pwm = pwm
        if abs(pwm - PWM_BUKA) <= PWM_TOLERANCE:
            self.buka()
        elif abs(pwm - PWM_TUTUP) <= PWM_TOLERANCE:
            self.tutup()
        else:
            print("pca_servo: servo%d PWM %d (no action)"
                  % (TRIGGER_SERVO, pwm))

    # ------------------------------------------------------------------ #
    # periodic tasks: stream request + heartbeat fail-safe
    # ------------------------------------------------------------------ #

    def idle_task(self):
        now = time.time()

        # make sure SERVO_OUTPUT_RAW is actually being streamed
        if (not self._seen_servo_output
                and self.mpstate.target_system
                and now - self._last_stream_request > STREAM_RETRY_S):
            self._last_stream_request = now
            try:
                self.master.mav.request_data_stream_send(
                    self.mpstate.target_system,
                    self.mpstate.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
                    STREAM_REQUEST_HZ,
                    1,  # start
                )
            except Exception:
                pass

        # fail-safe: FC link was up and is now lost -> release the servos
        if (self._seen_fc_heartbeat
                and not self._released
                and now - self._last_heartbeat > HEARTBEAT_TIMEOUT_S):
            print("pca_servo: heartbeat timeout - releasing servos (fail-safe)")
            self.lepas()

    # ------------------------------------------------------------------ #
    # console command for bench testing:  pcaservo <buka|tutup|lepas>
    # ------------------------------------------------------------------ #

    def cmd_pcaservo(self, args):
        if len(args) != 1 or args[0] not in ("buka", "tutup", "lepas"):
            print("usage: pcaservo <buka|tutup|lepas>")
            return
        getattr(self, args[0])()


def init(mpstate):
    """module initialisation"""
    return PCAServoModule(mpstate)
