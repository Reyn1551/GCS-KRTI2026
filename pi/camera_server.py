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
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

BOUNDARY = "frame"

INDEX_HTML = b"""<!doctype html><html><body style="margin:0;background:#000">
<img src="/stream" style="width:100%;height:100%;object-fit:contain"></body></html>"""


class Camera:
    """Grabs frames in a background thread and keeps the latest JPEG."""

    def __init__(self, device, width, height, fps, quality):
        dev = int(device) if str(device).isdigit() else device
        self.device = dev
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality
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

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(INDEX_HTML)))
            self.end_headers()
            self.wfile.write(INDEX_HTML)
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


def serve(device, port, width, height, fps, quality):
    camera = Camera(device, width, height, fps, quality)
    Handler.camera = camera
    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print("streaming %s on http://0.0.0.0:%d/stream" % (device, port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        camera.running = False
        if camera.cap is not None:
            camera.cap.release()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0", help="v4l2 device path or index")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--quality", type=int, default=80, help="JPEG quality 1-100")
    args = ap.parse_args()
    serve(args.device, args.port, args.width, args.height, args.fps, args.quality)
