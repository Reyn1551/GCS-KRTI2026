#!/usr/bin/env python3
"""
camera_server.py - MJPEG camera streamer for the Raspberry Pi.

Serves one camera as an MJPEG stream over HTTP, viewable in any browser
<img> tag (which is how the GCS web app embeds it).

Works for:
  - Logitech USB webcam (down-facing)
  - Intel RealSense D435i RGB stream (exposed as a standard UVC device -
    no librealsense needed when only RGB is required)

Find the right device nodes first:
    v4l2-ctl --list-devices

Usage:
    python3 camera_server.py --device /dev/video0 --port 8080
    python3 camera_server.py --device /dev/video4 --port 8081

Then open:  http://<pi-ip>:<port>/stream   (raw MJPEG)
            http://<pi-ip>:<port>/         (test page)
"""

import argparse
import fcntl
import json
import os
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2

BOUNDARY = "frame"

# ---- V4L2 exposure controls (linux/videodev2.h) ----
VIDIOC_QUERYCTRL = 0xC0445624   # _IOWR('V', 36, struct v4l2_queryctrl)
VIDIOC_G_CTRL = 0xC008561B      # _IOWR('V', 27, struct v4l2_control)
VIDIOC_S_CTRL = 0xC008561C      # _IOWR('V', 28, struct v4l2_control)
V4L2_CID_EXPOSURE_AUTO = 0x009A0901
V4L2_CID_EXPOSURE_ABSOLUTE = 0x009A0902
V4L2_EXPOSURE_AUTO = 3
V4L2_EXPOSURE_MANUAL = 1


def _query_exposure(device):
    """Return (min, max, value, is_auto) of the V4L2 exposure control, or None."""
    try:
        fd = os.open(device, os.O_RDONLY)
    except OSError:
        return None
    try:
        buf = bytearray(struct.pack("II32siiiiiII", V4L2_CID_EXPOSURE_ABSOLUTE, 0,
                                    b"", 0, 0, 0, 0, 0, 0, 0))
        fcntl.ioctl(fd, VIDIOC_QUERYCTRL, buf)
        _, _, _, cmin, cmax, _, cdef, _, _, _ = struct.unpack("II32siiiiiII", buf)
        gbuf = bytearray(struct.pack("II", V4L2_CID_EXPOSURE_ABSOLUTE, 0))
        if fcntl.ioctl(fd, VIDIOC_G_CTRL, gbuf) != 0:
            return None
        _, value = struct.unpack("II", gbuf)
        abuf = bytearray(struct.pack("II", V4L2_CID_EXPOSURE_AUTO, 0))
        is_auto = True
        if fcntl.ioctl(fd, VIDIOC_G_CTRL, abuf) == 0:
            _, auto = struct.unpack("II", abuf)
            is_auto = (auto != V4L2_EXPOSURE_MANUAL)
        return {"min": cmin, "max": cmax, "default": cdef,
                "value": value, "auto": is_auto}
    except OSError:
        return None
    finally:
        os.close(fd)


def _set_exposure(device, value):
    """Force manual exposure (µs). Returns updated state dict or None on error."""
    try:
        fd = os.open(device, os.O_RDONLY)
    except OSError:
        return None
    try:
        # 1. switch off auto exposure, 2. set absolute value
        abuf = bytearray(struct.pack("II", V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL))
        if fcntl.ioctl(fd, VIDIOC_S_CTRL, abuf) != 0:
            return None
        vbuf = bytearray(struct.pack("II", V4L2_CID_EXPOSURE_ABSOLUTE, int(value)))
        if fcntl.ioctl(fd, VIDIOC_S_CTRL, vbuf) != 0:
            return None
        return _query_exposure(device)
    except OSError:
        return None
    finally:
        os.close(fd)


INDEX_HTML = b"""<!doctype html><html><body style="margin:0;background:#000">
<img src="/stream" style="width:100%;height:100%;object-fit:contain"></body></html>"""


def detect_camera_device(target, port=None):
    """Auto-detect v4l2 device node based on camera model/type.

    - target='down' / 'logitech': Logitech webcam (facing down)
    - target='front' / 'realsense': Intel RealSense D435i RGB stream (facing front)
    - target='auto': inferred from port (8080 -> down/logitech, 8081 -> front/realsense)
    """
    target_str = str(target).lower().strip()

    if target_str == "auto":
        if port == 8080:
            target_str = "down"
        else:
            target_str = "front"

    if target_str in ("down", "logitech", "cam_down", "auto-down"):
        keywords = ["logitech", "brio", "c920", "c922", "c270", "c310", "c930", "046d"]
        fallback = "/dev/video0"
        label = "Logitech (Down)"
    elif target_str in ("front", "realsense", "d435", "d435i", "cam_front", "auto-front"):
        keywords = ["realsense", "d435", "intel"]
        fallback = "/dev/video4"
        label = "RealSense D435i RGB (Front)"
    else:
        if target_str.isdigit():
            return f"/dev/video{target_str}"
        return target

    # 1. Search /dev/v4l/by-id/
    by_id_path = "/dev/v4l/by-id"
    if os.path.exists(by_id_path):
        matches = []
        for fname in sorted(os.listdir(by_id_path)):
            if any(kw in fname.lower() for kw in keywords):
                full_path = os.path.realpath(os.path.join(by_id_path, fname))
                matches.append((fname, full_path))

        index0_matches = [m[1] for m in matches if "index0" in m[0]]
        for dev in index0_matches:
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if cap.isOpened():
                ok, frame = cap.read()
                cap.release()
                if ok and frame is not None:
                    return dev
                cap.release()

        for fname, dev in matches:
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if cap.isOpened():
                ok, frame = cap.read()
                cap.release()
                if ok and frame is not None:
                    return dev
                cap.release()

    # 2. Search /sys/class/video4linux/
    v4l_path = "/sys/class/video4linux"
    if os.path.exists(v4l_path):
        dev_nodes = sorted(
            os.listdir(v4l_path),
            key=lambda x: int(x.replace("video", "")) if x.replace("video", "").isdigit() else 999,
        )
        for dev_name in dev_nodes:
            name_file = os.path.join(v4l_path, dev_name, "name")
            if os.path.isfile(name_file):
                try:
                    with open(name_file, "r") as f:
                        title = f.read().strip()
                except Exception:
                    continue
                if any(kw in title.lower() for kw in keywords):
                    dev_path = f"/dev/{dev_name}"
                    cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
                    if cap.isOpened():
                        ok, frame = cap.read()
                        cap.release()
                        if ok and frame is not None:
                            return dev_path
                        cap.release()

    return fallback


class Camera:
    """Grabs frames in a background thread and keeps the latest JPEG."""

    def __init__(self, device, width, height, fps, quality, detector=None):
        dev = int(device) if str(device).isdigit() else device
        self.device = dev
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality
        self.detector = detector
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.cap = None
        self._open()
        threading.Thread(target=self._grab_loop, daemon=True).start()

    def _open(self):
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        # ask for MJPEG from the camera: much cheaper than raw YUYV over USB
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    def _grab_loop(self):
        failures = 0
        while self.running:
            ok, frame = self.cap.read() if self.cap is not None else (False, None)
            if not ok:
                failures += 1
                if failures % 10 == 0:
                    print("camera read failing, reopening device...")
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    time.sleep(1)
                    self._open()
                time.sleep(0.1)
                continue
            failures = 0
            if self.detector is not None:
                try:
                    frame = self.detector.annotate(frame)
                except Exception:
                    # never let detection break the stream; disable it
                    self.detector = None
                    print("detector failed, disabling detection (stream continues)")
            ok, jpg = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if ok:
                with self.lock:
                    self.frame = jpg.tobytes()

    def get_jpeg(self):
        with self.lock:
            return self.frame


class Handler(BaseHTTPRequestHandler):
    camera = None  # set by serve()

    def log_message(self, *args):
        pass  # keep the console clean

    def _send_json(self, payload, status=200):
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def handle_exposure(self):
        """GET /exposure            -> current state
           GET /exposure?value=N    -> set manual exposure to N µs
        """
        cam = self.camera
        device = getattr(cam, "device", None)
        if not device or str(device).isdigit():
            self._send_json({"error": "no camera device"}, status=400)
            return

        query = parse_qs(urlparse(self.path).query)
        if "value" in query:
            try:
                value = int(query["value"][0])
            except ValueError:
                self._send_json({"error": "invalid value"}, status=400)
                return
            state = _set_exposure(device, value)
            if state is None:
                self._send_json(
                    {"error": "exposure control not available on %s" % device},
                    status=400)
                return
            print("exposure set to %d µs" % state["value"])
            self._send_json({"ok": True, **state})
            return

        state = _query_exposure(device)
        if state is None:
            self._send_json(
                {"error": "exposure control not available on %s" % device},
                status=400)
            return
        self._send_json({"ok": True, **state})

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(INDEX_HTML)))
            self.end_headers()
            self.wfile.write(INDEX_HTML)
            return

        if self.path.startswith("/exposure"):
            self.handle_exposure()
            return

        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=%s" % BOUNDARY,
            )
            self.send_header("Cache-Control", "no-cache, no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                while True:
                    jpg = self.camera.get_jpeg()
                    if jpg is not None:
                        self.wfile.write(
                            b"--%s\r\nContent-Type: image/jpeg\r\n"
                            b"Content-Length: %d\r\n\r\n" % (BOUNDARY.encode(), len(jpg))
                        )
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(1.0 / 30)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        self.send_response(404)
        self.end_headers()


def serve(device, port, width, height, fps, quality, detect_model=None):
    resolved_device = detect_camera_device(device, port=port)
    print(f"Device setting: '{device}' -> Resolved to: '{resolved_device}'")

    detector = None
    if detect_model:
        try:
            from detector import Detector

            detector = Detector(detect_model)
        except Exception as e:
            detector = None
            print(f"WARNING: could not load detector model '{detect_model}': {e}")
            print("         continuing WITHOUT detection overlay")

    camera = Camera(resolved_device, width, height, fps, quality, detector=detector)
    Handler.camera = camera
    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print("streaming %s on http://0.0.0.0:%d/stream" % (resolved_device, port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        camera.running = False
        if camera.cap is not None:
            camera.cap.release()
        if detector is not None:
            detector.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="auto", help="v4l2 device path, index, or auto/down/logitech/front/realsense")
    ap.add_argument("--detect", choices=["down", "front", "logitech", "realsense"], help="Print auto-detected device path and exit")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--quality", type=int, default=80, help="JPEG quality 1-100")
    ap.add_argument("--detect-model", default=None, metavar="PATH",
                    help="HEF model path to enable object-detection overlay (dummy stage: "
                         "results are drawn on the stream but no action is taken)")
    args = ap.parse_args()

    if args.detect:
        dev = detect_camera_device(args.detect, port=args.port)
        print(dev)
        sys.exit(0)

    serve(args.device, args.port, args.width, args.height, args.fps, args.quality,
          detect_model=args.detect_model)

