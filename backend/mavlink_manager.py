"""
MAVLink connection manager for the KRTI ground control station.

Runs a background reader thread that:
  - listens for the MAVProxy UDP stream (default udpin:0.0.0.0:14550)
  - keeps a live telemetry snapshot
  - serves mission uploads (MISSION_COUNT / MISSION_REQUEST / MISSION_ACK)
  - tracks COMMAND_ACKs so REST calls can report real success/failure
  - sends a 1 Hz GCS heartbeat so ArduPilot sees a healthy GCS link
"""

import logging
import math
import threading
import time

from pymavlink import mavutil

log = logging.getLogger("gcs.mavlink")

MAV_RESULT_TEXT = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}


class NotConnectedError(Exception):
    """Raised when a command is issued with no heartbeat from the vehicle."""


class CommandRejectedError(Exception):
    """Raised when the vehicle NACKs a command."""


class MissionUploadError(Exception):
    """Raised when a mission upload fails or times out."""


class MAVLinkManager:
    HEARTBEAT_TIMEOUT = 5.0  # seconds without heartbeat -> link considered down

    def __init__(self, connection_string: str = "udpin:0.0.0.0:14550"):
        self.connection_string = connection_string
        self.master = None
        self.target_system = 0          # learned from the first autopilot heartbeat
        self.target_component = 1       # MAV_COMP_ID_AUTOPILOT1

        self._running = False
        self._reader_thread = None

        self._lock = threading.Lock()
        self._telemetry = self._default_telemetry()

        # COMMAND_ACK tracking
        self._cmd_lock = threading.Lock()
        self._pending_command = None
        self._cmd_ack_result = None
        self._cmd_ack_event = threading.Event()

        # Mission upload state machine
        self._mission_lock = threading.Lock()
        self._mission_items = None      # list of dicts while an upload is active
        self._active_mission_items = [] # last successfully uploaded mission items
        self._auto_closed_seqs = set()   # tracking seq numbers already auto-closed
        self._mission_error = None
        self._mission_done = threading.Event()

        # Parameter read/write
        self._param_lock = threading.Lock()
        self._params = {}               # name -> last known value
        self._param_waiters = {}        # name -> {"event": Event, "value": float|None}

    # ------------------------------------------------------------------ #
    # lifecycle
    # ------------------------------------------------------------------ #

    @staticmethod
    def _default_telemetry():
        return {
            "connected": False,
            "last_heartbeat": 0.0,
            "mode": "UNKNOWN",
            "armed": False,
            "lat": None,
            "lon": None,
            "home_lat": None,
            "home_lon": None,
            "alt_rel": None,
            "alt_msl": None,
            "heading": None,
            "groundspeed": None,
            "airspeed": None,
            "climb": None,
            "throttle": None,
            "gps_fix": None,
            "satellites": None,
            "battery_voltage": None,
            "battery_remaining": None,
            "roll": None,
            "pitch": None,
            "mission_seq": None,
            "last_status": None,
        }

    def start(self):
        self._running = True
        self._reader_thread = threading.Thread(
            target=self._run, name="mavlink-reader", daemon=True
        )
        self._reader_thread.start()

    def stop(self):
        self._running = False

    def get_telemetry(self):
        with self._lock:
            snap = dict(self._telemetry)
        snap["connected"] = (
            snap["last_heartbeat"] > 0
            and (time.time() - snap["last_heartbeat"]) < self.HEARTBEAT_TIMEOUT
        )
        return snap

    # ------------------------------------------------------------------ #
    # connection / reader loop
    # ------------------------------------------------------------------ #

    def _run(self):
        next_hb = 0.0
        while self._running:
            try:
                log.info("Opening MAVLink link: %s", self.connection_string)
                self.master = mavutil.mavlink_connection(
                    self.connection_string,
                    source_system=255,  # GCS system id
                    source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
                    dialect="ardupilotmega",
                )
                log.info("Link up, waiting for vehicle heartbeat...")
                while self._running:
                    now = time.time()
                    if now >= next_hb:
                        self._send_gcs_heartbeat()
                        next_hb = now + 1.0
                    msg = self.master.recv_match(blocking=True, timeout=0.5)
                    if msg is None:
                        continue
                    if msg.get_type() == "BAD_DATA":
                        continue
                    self._handle_message(msg)
            except Exception as exc:  # noqa: BLE001 - keep the link alive no matter what
                log.error("MAVLink link error: %s - retrying in 3 s", exc)
                self._set_telemetry(connected=False, last_heartbeat=0.0)
                time.sleep(3)

    def _send_gcs_heartbeat(self):
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
        except Exception:  # noqa: BLE001
            pass

    def _set_telemetry(self, **kwargs):
        with self._lock:
            self._telemetry.update(kwargs)

    # ------------------------------------------------------------------ #
    # message handling
    # ------------------------------------------------------------------ #

    def _handle_message(self, msg):
        mtype = msg.get_type()

        if mtype == "HEARTBEAT":
            # ignore heartbeats from other GCSs / companions
            if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                return
            if self.target_system == 0:
                self.target_system = msg.get_srcSystem()
                self.target_component = msg.get_srcComponent() or 1
                log.info(
                    "Vehicle found: sys=%s comp=%s", self.target_system, self.target_component
                )
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            try:
                mode = mavutil.mode_string_v10(msg)
            except Exception:  # noqa: BLE001
                mode = "UNKNOWN"
            self._set_telemetry(
                last_heartbeat=time.time(), connected=True, armed=armed, mode=mode
            )

        elif mtype == "GLOBAL_POSITION_INT":
            self._set_telemetry(
                lat=msg.lat / 1e7,
                lon=msg.lon / 1e7,
                alt_msl=msg.alt / 1000.0,
                alt_rel=msg.relative_alt / 1000.0,
                heading=msg.hdg / 100.0 if msg.hdg != 65535 else None,
                climb=-msg.vz / 100.0,
            )

        elif mtype == "VFR_HUD":
            self._set_telemetry(
                airspeed=msg.airspeed,
                groundspeed=msg.groundspeed,
                throttle=msg.throttle,
            )

        elif mtype == "GPS_RAW_INT":
            self._set_telemetry(
                gps_fix=msg.fix_type, satellites=msg.satellites_visible
            )

        elif mtype == "HOME_POSITION":
            self._set_telemetry(
                home_lat=msg.latitude / 1e7,
                home_lon=msg.longitude / 1e7,
            )

        elif mtype == "SYS_STATUS":
            self._set_telemetry(
                battery_voltage=msg.voltage_battery / 1000.0
                if msg.voltage_battery != 65535
                else None,
                battery_remaining=msg.battery_remaining
                if msg.battery_remaining != -1
                else None,
            )

        elif mtype == "ATTITUDE":
            self._set_telemetry(
                roll=math.degrees(msg.roll), pitch=math.degrees(msg.pitch)
            )

        elif mtype in ("MISSION_CURRENT", "MISSION_ITEM_REACHED"):
            self._set_telemetry(mission_seq=msg.seq)
            self._check_auto_close_servo(msg.seq)

        elif mtype == "COMMAND_ACK":
            with self._cmd_lock:
                if (
                    self._pending_command is not None
                    and msg.command == self._pending_command
                ):
                    self._cmd_ack_result = msg.result
                    self._cmd_ack_event.set()

        elif mtype == "STATUSTEXT":
            text = msg.text.rstrip("\x00 ")
            log.info("Vehicle status (sev %s): %s", msg.severity, text)
            self._set_telemetry(last_status=text)

        elif mtype in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            self._serve_mission_request(msg.seq)

        elif mtype == "MISSION_ACK":
            self._handle_mission_ack(msg.type)

        elif mtype == "PARAM_VALUE":
            self._handle_param_value(msg)

    # ------------------------------------------------------------------ #
    # commands
    # ------------------------------------------------------------------ #

    def _require_vehicle(self):
        if self.master is None or self.target_system == 0:
            raise NotConnectedError("No heartbeat from the vehicle yet")

    def _command_long(self, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0,
                      ack_timeout=3.0):
        """Send COMMAND_LONG and wait for the matching COMMAND_ACK."""
        self._require_vehicle()
        with self._cmd_lock:
            self._pending_command = command
            self._cmd_ack_result = None
            self._cmd_ack_event.clear()
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            command,
            0,  # confirmation
            p1, p2, p3, p4, p5, p6, p7,
        )
        # NOTE: wait happens outside the lock so the reader thread can
        # record the ACK while we block here
        got_ack = self._cmd_ack_event.wait(ack_timeout)
        with self._cmd_lock:
            self._pending_command = None
            result = self._cmd_ack_result

        if not got_ack:
            return {"ack": None, "result": None, "result_text": "NO_ACK"}
        text = MAV_RESULT_TEXT.get(result, str(result))
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            status = self._telemetry.get("last_status") or "no status text"
            raise CommandRejectedError(
                f"Command {command} rejected ({text}); vehicle: {status}"
            )
        return {"ack": True, "result": result, "result_text": text}

    def arm(self, do_arm: bool = True):
        return self._command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1 if do_arm else 0
        )

    def takeoff(self, altitude: float):
        return self._command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=float(altitude)
        )

    def land(self):
        return self._command_long(mavutil.mavlink.MAV_CMD_NAV_LAND)

    def rtl(self):
        return self._command_long(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def set_servo(self, servo: int, pwm: int):
        """MAV_CMD_DO_SET_SERVO -> picked up by the pca_servo MAVProxy module on the Pi."""
        return self._command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=int(servo), p2=int(pwm)
        )

    def set_mode(self, mode: str, timeout: float = 5.0):
        """Set flight mode; confirmed by watching the mode in the telemetry stream."""
        self._require_vehicle()
        mode = mode.upper()
        mapping = self.master.mode_mapping()
        if mapping is None or mode not in mapping:
            raise CommandRejectedError(f"Unknown mode '{mode}'")
        self.master.mav.set_mode_send(
            self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping[mode],
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.get_telemetry()["mode"] == mode:
                return {"ack": True, "mode": mode}
            time.sleep(0.2)
        raise CommandRejectedError(
            f"Mode change to {mode} not confirmed by the vehicle (mode stays: "
            f"{self.get_telemetry()['mode']})"
        )

    # ------------------------------------------------------------------ #
    # parameters (read / write / save to EEPROM)
    # ------------------------------------------------------------------ #

    def _handle_param_value(self, msg):
        name = msg.param_id.rstrip("\x00 ").upper()
        value = msg.param_value
        waiter = None
        with self._param_lock:
            self._params[name] = value
            waiter = self._param_waiters.get(name)
        if waiter is not None:
            waiter["value"] = value
            waiter["event"].set()

    def read_params(self, names, timeout: float = 4.0):
        """Read one or more parameters from the vehicle (PARAM_REQUEST_READ).

        All requests are sent in parallel and stragglers are retried once, so
        reading a full catalog is fast even on a congested link. Values are
        seeded from the last known cache, meaning known parameters still come
        back instantly the next refresh. Names the vehicle never answers
        (parameter missing from firmware, lost link) come back as None.
        """
        self._require_vehicle()
        names = [n for n in (str(raw).upper().strip() for raw in names) if n]

        # Seed from cache: values already seen streamed/stored are instant.
        out = {}
        with self._param_lock:
            for name in names:
                if name in self._params:
                    out[name] = self._params[name]

        unresolved = [n for n in names if n not in out]
        if not unresolved:
            return out

        def send_request(want):
            with self._param_lock:
                for name in want:
                    if name not in self._param_waiters:
                        self._param_waiters[name] = {
                            "event": threading.Event(), "value": None,
                        }
            for name in want:
                self.master.mav.param_request_read_send(
                    self.target_system,
                    self.target_component,
                    name.encode("ascii"),
                    -1,
                )

        def drain(want, deadline):
            remaining = list(want)
            while remaining and time.time() < deadline:
                time.sleep(0.2)
                done = []
                with self._param_lock:
                    for name in remaining:
                        waiter = self._param_waiters.get(name)
                        if waiter and waiter["event"].is_set():
                            out[name] = waiter["value"]
                            done.append(name)
                remaining = [n for n in remaining if n not in done]
            return remaining

        send_request(unresolved)
        missing = drain(unresolved, time.time() + timeout)
        if missing:
            # One retry for anything ArduPilot did not answer yet.
            send_request(missing)
            drain(missing, time.time() + timeout)

        with self._param_lock:
            for name in names:
                self._param_waiters.pop(name, None)
        return out

    def set_param(self, name, value, timeout: float = 3.0):
        """Set a parameter on the vehicle (PARAM_SET) and confirm the echo.

        ArduPilot replies with the stored value; a mismatch (or no reply)
        means the parameter was rejected.
        """
        self._require_vehicle()
        name = str(name).upper().strip()
        target = float(value)
        with self._param_lock:
            event = threading.Event()
            self._param_waiters[name] = {"event": event, "value": None}
        try:
            self.master.mav.param_set_send(
                self.target_system,
                self.target_component,
                name.encode("ascii"),
                target,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            if not event.wait(timeout):
                raise CommandRejectedError(f"No PARAM_VALUE reply for {name}")
            with self._param_lock:
                received = self._param_waiters[name]["value"]
            if received is None or abs(received - target) > 0.5:
                raise CommandRejectedError(
                    f"{name} rejected (autopilot kept {received})"
                )
            return {"name": name, "value": received}
        finally:
            with self._param_lock:
                self._param_waiters.pop(name, None)

    def save_params(self):
        """MAV_CMD_PREFLIGHT_STORAGE p1=2 -> write all params to EEPROM."""
        return self._command_long(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, p1=2
        )

    # ------------------------------------------------------------------ #
    # mission upload
    # ------------------------------------------------------------------ #

    def upload_mission(self, items, timeout: float = 30.0):
        """
        Upload a full mission (list of dicts with frame/command/params/x/y/z).
        Implements the MAVLink mission upload state machine.
        """
        self._require_vehicle()
        with self._mission_lock:
            self._mission_items = list(items)
            self._mission_error = None
            self._mission_done.clear()
            count = len(items)
        self.master.mav.mission_count_send(
            self.target_system, self.target_component, count
        )
        log.info("Mission upload started: %d items", count)
        # NOTE: wait happens outside the lock so the reader thread can
        # serve MISSION_REQUESTs while we block here
        finished = self._mission_done.wait(timeout)
        with self._mission_lock:
            self._mission_items = None
            error = self._mission_error
        if not finished:
            raise MissionUploadError("Mission upload timed out")
        if error:
            raise MissionUploadError(error)
        log.info("Mission upload complete")
        with self._mission_lock:
            self._active_mission_items = list(items)
            self._auto_closed_seqs.clear()
        return {"uploaded": len(items)}

    def _check_auto_close_servo(self, seq: int):
        with self._mission_lock:
            if not self._active_mission_items:
                return
            for idx in range(min(seq + 1, len(self._active_mission_items))):
                if idx in self._auto_closed_seqs:
                    continue
                it = self._active_mission_items[idx]
                if (
                    it.get("command") == mavutil.mavlink.MAV_CMD_DO_SET_SERVO
                    and it.get("auto_close")
                ):
                    self._auto_closed_seqs.add(idx)
                    servo = int(it.get("param1", 10))
                    close_pwm = int(it.get("close_pwm", 1100))
                    delay_sec = float(it.get("close_delay", 2.0))

                    def _do_auto_close(s=servo, cp=close_pwm, d=delay_sec, item_seq=idx):
                        log.info(
                            "Auto-close trigger (seq #%d): waiting %.1fs before closing servo %d (PWM %d) while continuing mission...",
                            item_seq, d, s, cp,
                        )
                        time.sleep(d)
                        try:
                            self.set_servo(s, cp)
                            log.info(
                                "Servo %d auto-closed to PWM %d successfully after %.1fs delay while in flight!",
                                s, cp, d,
                            )
                        except Exception as err:
                            log.error("Failed auto-closing servo %d: %s", s, err)

                    import threading
                    threading.Thread(
                        target=_do_auto_close,
                        name=f"servo-autoclose-{idx}",
                        daemon=True,
                    ).start()

    def _serve_mission_request(self, seq):
        with self._mission_lock:
            if self._mission_items is None or seq >= len(self._mission_items):
                return
            it = self._mission_items[seq]
            self.master.mav.mission_item_int_send(
                self.target_system,
                self.target_component,
                seq,
                it["frame"],
                it["command"],
                it.get("current", 0),
                it.get("autocontinue", 1),
                it.get("param1", 0.0),
                it.get("param2", 0.0),
                it.get("param3", 0.0),
                it.get("param4", 0.0),
                int(it.get("x", 0)),
                int(it.get("y", 0)),
                float(it.get("z", 0.0)),
            )

    def _handle_mission_ack(self, ack_type):
        with self._mission_lock:
            if self._mission_items is None:
                return
            if ack_type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                self._mission_error = None
            else:
                self._mission_error = f"Vehicle NACKed mission (ack type {ack_type})"
            self._mission_done.set()
