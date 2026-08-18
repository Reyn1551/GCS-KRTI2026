"""
KRTI Ground Control Station - FastAPI backend.

Serves the web UI, streams telemetry over a WebSocket, and exposes REST
endpoints for arming, takeoff, landing, mission upload and servo control.

Run:  uvicorn main:app --host 0.0.0.0 --port 8000
"""

import asyncio
import hashlib
import logging
import math
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal, Optional

import httpx

from fastapi import FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from mavlink_manager import (
    CommandRejectedError,
    MAVLinkManager,
    MissionUploadError,
    NotConnectedError,
)
from pymavlink import mavutil

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s"
)
log = logging.getLogger("gcs.api")

MAVLINK_CONN = os.environ.get("MAVLINK_CONN", "udpin:0.0.0.0:14550")
FRONTEND_DIR = Path(__file__).resolve().parent.parent / "frontend"

manager = MAVLinkManager(MAVLINK_CONN)


@asynccontextmanager
async def lifespan(app: FastAPI):
    manager.start()
    log.info("MAVLink manager started on %s", MAVLINK_CONN)
    yield
    manager.stop()


app = FastAPI(title="KRTI Ground Control Station", lifespan=lifespan)


# ---------------------------------------------------------------------- #
# request models
# ---------------------------------------------------------------------- #

class TakeoffRequest(BaseModel):
    altitude: float = Field(default=1.0, gt=0.1, le=100.0)


class ModeRequest(BaseModel):
    mode: str = Field(min_length=2, max_length=20)


class ServoRequest(BaseModel):
    servo: int = Field(ge=1, le=16, description="MAVLink servo number (mapped on the Pi)")
    pwm: int = Field(ge=500, le=2500, description="Pulse width in microseconds")


class MissionItemIn(BaseModel):
    type: Literal["waypoint", "servo", "delay", "takeoff", "land", "arm", "disarm"]
    # waypoint fields
    lat: Optional[float] = Field(default=None, ge=-90, le=90)
    lon: Optional[float] = Field(default=None, ge=-180, le=180)
    alt: float = Field(default=1.0, gt=0, le=200)
    radius: Optional[float] = Field(default=2.0, ge=0.05, le=50)
    # servo fields
    servo: Optional[int] = Field(default=None, ge=1, le=16)
    pwm: Optional[int] = Field(default=None, ge=500, le=2500)
    auto_close: Optional[bool] = Field(default=False)
    close_delay: Optional[float] = Field(default=2.0, ge=0.1, le=3600)
    close_pwm: Optional[int] = Field(default=1100, ge=500, le=2500)
    # delay fields (seconds)
    delay: Optional[float] = Field(default=0.0, ge=0, le=3600)


class MissionUploadRequest(BaseModel):
    items: list[MissionItemIn] = Field(min_length=1, max_length=200)


class ParamReadRequest(BaseModel):
    names: list[str] = Field(min_length=1, max_length=200)


class ParamSetRequest(BaseModel):
    name: str = Field(min_length=2, max_length=20)
    value: float = Field(ge=-1e9, le=1e9)


# ---------------------------------------------------------------------- #
# ArduPilot parameter catalog (metadata shown in the CONFIG tab)
# Values themselves are always read live from the vehicle.
# ---------------------------------------------------------------------- #

PARAM_DEFS = [
    # --- Waypoint Navigation (WPNAV) ---
    {
        "group": "NAVIGASI WAYPOINT",
        "name": "WPNAV_SPEED",
        "title": "Kecepatan Jelajah Waypoint",
        "desc": "Kecepatan horizontal maksimum yang dipertahankan drone saat terbang otomatis antar waypoint misi.",
        "unit": "cm/s", "min": 20, "max": 2000, "step": 50, "default": 500,
    },
    {
        "group": "NAVIGASI WAYPOINT",
        "name": "WPNAV_SPEED_UP",
        "title": "Kecepatan Naiknya Drone (Climb)",
        "desc": "Kecepatan pemanjatan vertikal maksimum saat drone bergerak naik menuju altitude target.",
        "unit": "cm/s", "min": 10, "max": 1000, "step": 50, "default": 250,
    },
    {
        "group": "NAVIGASI WAYPOINT",
        "name": "WPNAV_SPEED_DN",
        "title": "Kecepatan Turunnya Drone (Descent)",
        "desc": "Kecepatan penurunan vertikal maksimum saat drone bergerak turun antar waypoint.",
        "unit": "cm/s", "min": 10, "max": 500, "step": 10, "default": 150,
    },
    {
        "group": "NAVIGASI WAYPOINT",
        "name": "WPNAV_ACCEL",
        "title": "Akselerasi Horizontal Navigasi",
        "desc": "Tingkat akselerasi & pengereman horizontal saat misi. Nilai lebih tinggi membuat gerakan lebih responsif namun lebih kaku.",
        "unit": "cm/s²", "min": 50, "max": 500, "step": 10, "default": 250,
    },
    {
        "group": "NAVIGASI WAYPOINT",
        "name": "WPNAV_ACCEL_Z",
        "title": "Akselerasi Vertikal Navigasi",
        "desc": "Tingkat akselerasi gerakan naik atau turun drone saat perpindahan altitude.",
        "unit": "cm/s²", "min": 50, "max": 500, "step": 10, "default": 100,
    },
    {
        "group": "NAVIGASI WAYPOINT",
        "name": "WPNAV_JERK",
        "title": "Kehalusan Transisi Akselerasi (Jerk)",
        "desc": "Batas perubahan akselerasi horizontal (jerk). Nilai 0 menggunakan standar default firmware.",
        "unit": "m/s³", "min": 0, "max": 20, "step": 1, "default": 10,
    },
    {
        "group": "NAVIGASI WAYPOINT",
        "name": "WPNAV_RADIUS",
        "title": "Radius Toleransi Waypoint (Acceptance)",
        "desc": "Jarak minimal drone ke koordinat waypoint agar dianggap sudah tercapai sebelum lanjut ke WP berikutnya.",
        "unit": "cm", "min": 5, "max": 1000, "step": 1, "default": 200,
    },
    {
        "group": "NAVIGASI WAYPOINT",
        "name": "WPNAV_ALT_MIN",
        "title": "Ketinggian Minimum Navigasi",
        "desc": "Batas ketinggian minimal saat takeoff/landing relatif terhadap titik Home.",
        "unit": "cm", "min": 0, "max": 10000, "step": 10, "default": 0,
    },
    {
        "group": "NAVIGASI WAYPOINT",
        "name": "WPNAV_RFND_USE",
        "title": "Terrain Following (Sensor Jarak)",
        "desc": "Aktifkan penggunaan Rangefinder/Lidar untuk mengikuti kontur permukaan tanah (0=Nonaktif, 1=Aktif).",
        "unit": "", "min": 0, "max": 1, "step": 1, "default": 1,
    },

    # --- Mode Loiter dalam Misi ---
    {
        "group": "LOITER & HOVERING",
        "name": "WPNAV_LOITER_SPEED",
        "title": "Kecepatan Maksimum Mode Loiter",
        "desc": "Kecepatan horizontal maksimum saat drone ditahan pada posisi LOITER.",
        "unit": "cm/s", "min": 20, "max": 2000, "step": 50, "default": 1200,
    },
    {
        "group": "LOITER & HOVERING",
        "name": "WPNAV_LOITER_RADIUS",
        "title": "Radius Lingkaran Loiter",
        "desc": "Ukuran radius penahanan posisi loiter melingkar.",
        "unit": "cm", "min": 25, "max": 2000, "step": 25, "default": 1000,
    },
    {
        "group": "LOITER & HOVERING",
        "name": "WPNAV_LOITER_ACCEL",
        "title": "Akselerasi Meredam Loiter",
        "desc": "Tingkat pengereman horizontal saat drone berhenti di titik Loiter.",
        "unit": "cm/s²", "min": 50, "max": 1500, "step": 10, "default": 500,
    },
    {
        "group": "LOITER & HOVERING",
        "name": "WPNAV_LOITER_JERK",
        "title": "Kehalusan Akselerasi Loiter",
        "desc": "Batas jerk saat penahanan posisi Loiter. 0 = default firmware.",
        "unit": "m/s³", "min": 0, "max": 20, "step": 1, "default": 10,
    },
    {
        "group": "LOITER & HOVERING",
        "name": "WPNAV_LOITER_MIN_T",
        "title": "Durasi Diam Minimal Loiter",
        "desc": "Durasi penahanan posisi minimal (dalam detik) sebelum diperbolehkan lanjut.",
        "unit": "s", "min": 0, "max": 120, "step": 1, "default": 0,
    },
    {
        "group": "LOITER & HOVERING",
        "name": "WPNAV_LOITER_MAX_T",
        "title": "Durasi Diam Maksimal Loiter",
        "desc": "Durasi penahanan posisi maksimal (dalam detik). 0 = tanpa batas waktu.",
        "unit": "s", "min": 0, "max": 120, "step": 1, "default": 0,
    },

    # --- Mode Return To Launch (RTL) ---
    {
        "group": "RETURN TO LAUNCH (RTL)",
        "name": "RTL_SPEED",
        "title": "Kecepatan Terbang Pulang (RTL)",
        "desc": "Kecepatan horizontal saat terbang kembali ke titik Home. (0 = menggunakan WPNAV_SPEED).",
        "unit": "cm/s", "min": 0, "max": 2000, "step": 50, "default": 0,
    },
    {
        "group": "RETURN TO LAUNCH (RTL)",
        "name": "RTL_ALT",
        "title": "Altitude Ketinggian Pulang (RTL)",
        "desc": "Ketinggian aman drone di atas home sebelum terbang kembali. Jika posisi saat ini lebih tinggi, drone akan RTL di ketinggian sekarang.",
        "unit": "cm", "min": 200, "max": 8000, "step": 100, "default": 1500,
    },
    {
        "group": "RETURN TO LAUNCH (RTL)",
        "name": "RTL_ALT_FINAL",
        "title": "Ketinggian Akhir Setelah Sampai Home",
        "desc": "Ketinggian drone setelah sampai di atas titik Home. Set ke 0 untuk mendarat otomatis.",
        "unit": "cm", "min": 0, "max": 1000, "step": 1, "default": 0,
    },
    {
        "group": "RETURN TO LAUNCH (RTL)",
        "name": "RTL_CLIMB_MIN",
        "title": "Kenaikan Awal Minimal saat RTL",
        "desc": "Tambahan kenaikan tinggi minimal (cm) pada fase pertama saat perintah RTL diaktifkan.",
        "unit": "cm", "min": 0, "max": 3000, "step": 10, "default": 0,
    },
    {
        "group": "RETURN TO LAUNCH (RTL)",
        "name": "RTL_LOIT_TIME",
        "title": "Waktu Diam di Atas Home (RTL)",
        "desc": "Durasi waktu penahanan (hovering) di atas titik Home sebelum memulai penurunan/landing.",
        "unit": "ms", "min": 0, "max": 60000, "step": 1000, "default": 5000,
    },

    # --- Mode Landing ---
    {
        "group": "PENDARATAN (LANDING)",
        "name": "LAND_SPEED",
        "title": "Kecepatan Sentuh Tanah (Land Akhir)",
        "desc": "Kecepatan turun akhir saat drone mendekati permukaan tanah untuk pendaratan mulus.",
        "unit": "cm/s", "min": 30, "max": 200, "step": 10, "default": 50,
    },
    {
        "group": "PENDARATAN (LANDING)",
        "name": "LAND_SPEED_HIGH",
        "title": "Kecepatan Turun Awal (Land Atas)",
        "desc": "Kecepatan penurunan awal dari ketinggian tinggi saat perintah LAND (0 = mengikuti WPNAV_SPEED_DN).",
        "unit": "cm/s", "min": 0, "max": 500, "step": 10, "default": 0,
    },

    # --- Mode Behavior & Safety ---
    {
        "group": "ORIENTASI & KESELAMATAN",
        "name": "WP_YAW_BEHAVIOR",
        "title": "Perilaku Hadap Drone (Yaw)",
        "desc": "Mode arah hadap moncong drone saat terbang misi: 0=Kunci orientasi awal, 1=Menghadap ke WP berikutnya, 2=Menghadap WP kecuali saat RTL, 3=Mengikuti arah GPS.",
        "unit": "", "min": 0, "max": 3, "step": 1, "default": 1,
    },
    {
        "group": "ORIENTASI & KESELAMATAN",
        "name": "FS_THR_ENABLE",
        "title": "Failsafe Sinyal Remote (Throttle)",
        "desc": "Aksi keselamatan jika sinyal pemancar terputus: 0=Matikan Failsafe, 1=Terbang Pulang (RTL), 2=Lanjutkan Misi AUTO.",
        "unit": "", "min": 0, "max": 2, "step": 1, "default": 1,
    },
    {
        "group": "ORIENTASI & KESELAMATAN",
        "name": "FS_BATT_ENABLE",
        "title": "Failsafe Baterai Lemah",
        "desc": "Aksi otomatis jika voltase baterai di bawah batas kritis: 0=Nonaktif, 1=Mendarat di tempat (LAND), 2=Terbang Pulang (RTL).",
        "unit": "", "min": 0, "max": 2, "step": 1, "default": 2,
    },
]


# ---------------------------------------------------------------------- #
# helpers
# ---------------------------------------------------------------------- #

def _handle_errors(fn):
    try:
        return fn()
    except NotConnectedError as exc:
        raise HTTPException(status_code=503, detail=str(exc))
    except CommandRejectedError as exc:
        raise HTTPException(status_code=409, detail=str(exc))
    except MissionUploadError as exc:
        raise HTTPException(status_code=504, detail=str(exc))


def _build_mission_items(req: MissionUploadRequest):
    """Convert UI mission items into MAVLink mission item dicts.

    Item 0 is the ArduPilot home placeholder (current position if known).
    If the mission does not start with a TAKEOFF item, one is inserted
    automatically (ArduPilot will not auto-takeoff from a waypoint item).
    ARM/DISARM items are NOT valid ArduPilot mission commands (they would be
    NACKed with MAV_MISSION_UNSUPPORTED), so they are collected separately
    and executed as COMMAND_LONG after the upload.
    Returns (items, auto_takeoff_inserted, pending_actions).
    """
    mav = mavutil.mavlink
    tel = manager.get_telemetry()
    home_lat = tel["lat"] if tel["lat"] is not None else 0.0
    home_lon = tel["lon"] if tel["lon"] is not None else 0.0

    items = [
        dict(
            frame=mav.MAV_FRAME_GLOBAL,
            command=mav.MAV_CMD_NAV_WAYPOINT,
            current=0,
            autocontinue=1,
            param1=0, param2=0, param3=0, param4=0,
            x=int(home_lat * 1e7), y=int(home_lon * 1e7), z=0.0,
        )
    ]

    has_takeoff = any(it.type == "takeoff" for it in req.items)
    auto_takeoff = False
    pending_actions = []
    if not has_takeoff:
        first_alt = next(
            (it.alt for it in req.items if it.type == "waypoint" and it.alt), None
        )
        takeoff_alt = first_alt if first_alt is not None else 10.0
        auto_takeoff = True
        items.append(
            dict(
                frame=mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                command=mav.MAV_CMD_NAV_TAKEOFF,
                current=0,
                autocontinue=1,
                param1=0, param2=0, param3=0, param4=math.nan,
                x=0, y=0, z=float(takeoff_alt),
            )
        )

    for it in req.items:
        if it.type == "waypoint":
            if it.lat is None or it.lon is None:
                raise HTTPException(400, "Waypoint item missing lat/lon")
            items.append(
                dict(
                    frame=mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    command=mav.MAV_CMD_NAV_WAYPOINT,
                    current=0,
                    autocontinue=1,
                    param1=float(it.delay or 0),  # hold time (seconds)
                    param2=2,                      # acceptance radius (m)
                    param3=0,                      # pass through
                    param4=math.nan,               # yaw: ignore
                    x=int(it.lat * 1e7), y=int(it.lon * 1e7), z=float(it.alt),
                )
            )
        elif it.type == "servo":
            if it.servo is None or it.pwm is None:
                raise HTTPException(400, "Servo item missing servo/pwm")
            items.append(
                dict(
                    frame=mav.MAV_FRAME_MISSION,
                    command=mav.MAV_CMD_DO_SET_SERVO,
                    current=0,
                    autocontinue=1,
                    param1=float(it.servo),
                    param2=float(it.pwm),
                    param3=0, param4=0,
                    x=0, y=0, z=0.0,
                    auto_close=bool(it.auto_close),
                    close_delay=float(it.close_delay or 2.0),
                    close_pwm=int(it.close_pwm or 1100),
                )
            )
        elif it.type == "delay":
            delay_sec = float(it.delay or 0)
            items.append(
                dict(
                    frame=mav.MAV_FRAME_MISSION,
                    command=mav.MAV_CMD_NAV_DELAY,
                    current=0,
                    autocontinue=1,
                    param1=delay_sec,  # delay duration (seconds)
                    param2=-1.0,        # hour (-1 ignore)
                    param3=-1.0,        # min (-1 ignore)
                    param4=-1.0,        # sec (-1 ignore)
                    x=0, y=0, z=0.0,
                )
            )
        elif it.type == "takeoff":
            items.append(
                dict(
                    frame=mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    command=mav.MAV_CMD_NAV_TAKEOFF,
                    current=0,
                    autocontinue=1,
                    param1=0, param2=0, param3=0, param4=math.nan,
                    x=0, y=0, z=float(it.alt),
                )
            )
        elif it.type == "land":
            items.append(
                dict(
                    frame=mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    command=mav.MAV_CMD_NAV_LAND,
                    current=0,
                    autocontinue=1,
                    param1=0, param2=0, param3=0, param4=math.nan,
                    x=0, y=0, z=0.0,
                )
            )
        elif it.type == "arm":
            pending_actions.append("arm")
        elif it.type == "disarm":
            pending_actions.append("disarm")
    return items, auto_takeoff, pending_actions


# ---------------------------------------------------------------------- #
# REST API
# ---------------------------------------------------------------------- #

@app.get("/api/telemetry")
def telemetry():
    return manager.get_telemetry()


@app.post("/api/arm")
def arm():
    return _handle_errors(lambda: manager.arm(True))


@app.post("/api/disarm")
def disarm():
    return _handle_errors(lambda: manager.arm(False))


@app.post("/api/takeoff")
def takeoff(req: TakeoffRequest):
    """Guided takeoff: set GUIDED, arm, wait for spool-up, then climb."""
    def do_takeoff():
        manager.set_mode("GUIDED")
        arm_result = manager.arm(True)
        time.sleep(1.5)  # let the motors spin up before the takeoff command
        tk_result = manager.takeoff(req.altitude)
        return {"arm": arm_result, "takeoff": tk_result}
    return _handle_errors(do_takeoff)


@app.post("/api/land")
def land():
    return _handle_errors(manager.land)


@app.post("/api/rtl")
def rtl():
    return _handle_errors(manager.rtl)


@app.post("/api/mode")
def set_mode(req: ModeRequest):
    return _handle_errors(lambda: manager.set_mode(req.mode))


# ---------------------------------------------------------------------- #
# parameter configuration (WPNAV / RTL / LAND ...)
# ---------------------------------------------------------------------- #

@app.get("/api/params/defs")
def params_defs():
    return {"params": PARAM_DEFS}


@app.post("/api/params/read")
def params_read(req: ParamReadRequest):
    """Read the given parameters live from the vehicle (PARAM_REQUEST_READ)."""
    return _handle_errors(lambda: {"values": manager.read_params(req.names)})


@app.post("/api/params/set")
def params_set(req: ParamSetRequest):
    """Set one parameter on the vehicle (PARAM_SET, RAM only)."""
    return _handle_errors(lambda: manager.set_param(req.name, req.value))


@app.post("/api/params/save")
def params_save():
    """Write all RAM parameters to the autopilot EEPROM (PREFLIGHT_STORAGE)."""
    return _handle_errors(manager.save_params)


@app.post("/api/mission/upload")
def mission_upload(req: MissionUploadRequest):
    items, auto_takeoff, actions = _build_mission_items(req)
    result = _handle_errors(lambda: manager.upload_mission(items))
    executed = []
    for action in actions:
        def run(action=action):
            if action == "arm":
                return manager.arm(True)
            return manager.arm(False)
        executed.append(_handle_errors(run))
    return {**result, "auto_takeoff": auto_takeoff, "actions": executed}


@app.post("/api/mission/start")
def mission_start():
    """Arm the motors, then switch to AUTO; the vehicle flies the uploaded mission."""
    def do_start():
        arm_result = manager.arm(True)
        time.sleep(0.5)  # let arming settle before switching modes
        mode_result = manager.set_mode("AUTO")
        return {"arm": arm_result, "mode": mode_result}
    return _handle_errors(do_start)


@app.post("/api/mission/pause")
def mission_pause():
    """Hold the current position (LOITER) without aborting the mission."""
    return _handle_errors(lambda: manager.set_mode("LOITER"))


@app.post("/api/servo")
def servo(req: ServoRequest):
    return _handle_errors(lambda: manager.set_servo(req.servo, req.pwm))


# ---------------------------------------------------------------------- #
# tile proxy (offline-capable map tiles)
# ---------------------------------------------------------------------- #

TILE_CACHE_DIR = Path(__file__).resolve().parent.parent / "tile_cache"
TILE_URL_TEMPLATE = os.environ.get(
    "TILE_URL",
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
)
TILE_USER_AGENT = "KRTI-GCS/1.0"
TILE_MAX_ZOOM = 19
TILE_MAX_CACHE = 500 * 1024 * 1024  # 500 MB


@app.get("/api/tiles/{z}/{x}/{y}")
async def tile_proxy(z: int, x: int, y: int):
    if z < 0 or z > TILE_MAX_ZOOM:
        raise HTTPException(400, "zoom out of range")
    if x < 0 or y < 0:
        raise HTTPException(400, "invalid tile coords")

    cache_key = f"{z}/{x}/{y}"
    cache_hash = hashlib.md5(cache_key.encode()).hexdigest()
    cache_file = TILE_CACHE_DIR / f"{cache_hash}.png"
    cache_meta = TILE_CACHE_DIR / f"{cache_hash}.meta"

    # serve from cache
    if cache_file.exists():
        headers = {}
        if cache_meta.exists():
            try:
                raw = cache_meta.read_text()
                _, _, _, _, age = raw.split("|", 4)
                headers["X-Tile-Cache"] = "HIT"
                headers["X-Tile-Age"] = age
            except Exception:
                pass
        return Response(
            content=cache_file.read_bytes(),
            media_type="image/png",
            headers=headers,
        )

    # fetch from upstream
    url = TILE_URL_TEMPLATE.replace("{z}", str(z)).replace("{x}", str(x)).replace("{y}", str(y))
    try:
        async with httpx.AsyncClient() as client:
            resp = await client.get(url, headers={"User-Agent": TILE_USER_AGENT}, timeout=10)
        resp.raise_for_status()
    except Exception as exc:
        raise HTTPException(502, f"tile upstream error: {exc}")

    body = resp.content
    media_type = resp.headers.get("Content-Type", "image/png")

    # write to cache
    try:
        TILE_CACHE_DIR.mkdir(parents=True, exist_ok=True)
        cache_file.write_bytes(body)
        cache_meta.write_text(f"{z}|{x}|{y}|{time.time()}|0")
        # evict oldest if over limit
        _evict_tile_cache()
    except Exception:
        pass

    return Response(content=body, media_type=media_type, headers={"X-Tile-Cache": "MISS"})


def _evict_tile_cache():
    try:
        total = sum(f.stat().st_size for f in TILE_CACHE_DIR.iterdir() if f.suffix == ".png")
        if total < TILE_MAX_CACHE:
            return
        files = sorted(
            [f for f in TILE_CACHE_DIR.iterdir() if f.suffix == ".png"],
            key=lambda f: f.stat().st_mtime,
        )
        while total > TILE_MAX_CACHE and files:
            f = files.pop(0)
            total -= f.stat().st_size
            f.unlink(missing_ok=True)
            meta = f.with_suffix(".meta")
            meta.unlink(missing_ok=True)
    except Exception:
        pass


# ---------------------------------------------------------------------- #
# telemetry websocket
# ---------------------------------------------------------------------- #

@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await ws.send_json(manager.get_telemetry())
            await asyncio.sleep(0.2)
    except (WebSocketDisconnect, RuntimeError, Exception):
        pass


# ---------------------------------------------------------------------- #
# frontend (mounted last so /api and /ws keep priority)
# ---------------------------------------------------------------------- #

app.mount("/", StaticFiles(directory=str(FRONTEND_DIR), html=True), name="frontend")
