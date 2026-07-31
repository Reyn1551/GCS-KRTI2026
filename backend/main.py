"""
KRTI Ground Control Station - FastAPI backend.

Serves the web UI, streams telemetry over a WebSocket, and exposes REST
endpoints for arming, takeoff, landing, mission upload and servo control.

Run:  uvicorn main:app --host 0.0.0.0 --port 8000
"""

import asyncio
import logging
import math
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal, Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
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
    altitude: float = Field(default=5.0, gt=0.5, le=100.0)


class ModeRequest(BaseModel):
    mode: str = Field(min_length=2, max_length=20)


class ServoRequest(BaseModel):
    servo: int = Field(ge=1, le=16, description="MAVLink servo number (mapped on the Pi)")
    pwm: int = Field(ge=500, le=2500, description="Pulse width in microseconds")


class MissionItemIn(BaseModel):
    type: Literal["waypoint", "servo"]
    # waypoint fields
    lat: Optional[float] = Field(default=None, ge=-90, le=90)
    lon: Optional[float] = Field(default=None, ge=-180, le=180)
    alt: float = Field(default=10.0, gt=0, le=200)
    # servo fields
    servo: Optional[int] = Field(default=None, ge=1, le=16)
    pwm: Optional[int] = Field(default=None, ge=500, le=2500)


class MissionUploadRequest(BaseModel):
    items: list[MissionItemIn] = Field(min_length=1, max_length=200)


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
                    param1=0,           # hold time (s)
                    param2=2,           # acceptance radius (m)
                    param3=0,           # pass through
                    param4=math.nan,    # yaw: ignore
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
                )
            )
    return items


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


@app.post("/api/mission/upload")
def mission_upload(req: MissionUploadRequest):
    items = _build_mission_items(req)
    return _handle_errors(lambda: manager.upload_mission(items))


@app.post("/api/mission/start")
def mission_start():
    """Switch to AUTO; the vehicle flies the uploaded mission."""
    return _handle_errors(lambda: manager.set_mode("AUTO"))


@app.post("/api/mission/pause")
def mission_pause():
    """Hold the current position (LOITER) without aborting the mission."""
    return _handle_errors(lambda: manager.set_mode("LOITER"))


@app.post("/api/servo")
def servo(req: ServoRequest):
    return _handle_errors(lambda: manager.set_servo(req.servo, req.pwm))


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
    except WebSocketDisconnect:
        pass


# ---------------------------------------------------------------------- #
# frontend (mounted last so /api and /ws keep priority)
# ---------------------------------------------------------------------- #

app.mount("/", StaticFiles(directory=str(FRONTEND_DIR), html=True), name="frontend")
