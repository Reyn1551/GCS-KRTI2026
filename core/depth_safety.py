"""
KP2026 DepthSafetyMonitor — RealSense D435i stereo depth anti-collision.

Two modes:
  1. Standalone — owns its own RealSense pipeline (depth_viewer.py test tool)
  2. External (push_frame) — receives pre-aligned depth frames from a shared
     pipeline, runs processing in background thread (gate_mission.py integration)

Thread-safe access to:
  - Forward path obstacle detection
  - Gate aperture clearance verification
  - Depth-validated distance measurement
  - Depth noise estimation + colormap for HUD
"""

import logging
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None  # type: ignore[assignment]

try:
    import pyrealsense2 as rs

    HAS_REALSENSE = True
except ImportError:
    rs = None  # type: ignore[assignment]
    HAS_REALSENSE = False

import config as cfg

logger = logging.getLogger(__name__)


@dataclass
class ClearanceResult:
    """Gate aperture clearance check result."""

    is_clear: bool = False
    is_symmetric: bool = False
    center_depth: float = 0.0
    left_depth: float = 0.0
    right_depth: float = 0.0
    top_depth: float = 0.0
    bottom_depth: float = 0.0
    edge_ratio: float = 0.0
    warning: str = ""


@dataclass
class SafetyState:
    """Thread-safe snapshot of depth safety system state."""

    forward_clear: bool = True
    forward_depth: float = 99.0
    gate_clearance: Optional[ClearanceResult] = None
    measured_distance: float = 99.0
    depth_available: bool = False
    noise_level: float = 0.0
    frame_timestamp: float = 0.0
    fps: float = 0.0
    aperture_valid: bool = False
    aperture_cx: float = 0.0
    aperture_cy: float = 0.0
    aperture_left: float = 0.0
    aperture_right: float = 0.0
    aperture_top: float = 0.0
    aperture_bottom: float = 0.0
    aperture_w: float = 0.0
    aperture_h: float = 0.0
    aperture_clearance_m: float = 99.0


class ApertureDetector:
    """
    Precisely locates gate opening edges using depth gradient analysis.

    Runs inside the DepthSafetyMonitor thread — zero main-thread overhead.
    """

    def __init__(
        self,
        depth_threshold_mm: float = 1500,
        min_opening_ratio: float = 0.15,
        edge_consensus_ratio: float = 0.4,
        fx: float = cfg.FX,
        fy: float = cfg.FY,
        camera_width: int = cfg.CAMERA_WIDTH,
        camera_height: int = cfg.CAMERA_HEIGHT,
    ):
        self.depth_threshold_mm = depth_threshold_mm
        self.min_opening_ratio = min_opening_ratio
        self.edge_consensus_ratio = edge_consensus_ratio
        self.fx = fx
        self.fy = fy
        self.camera_width = camera_width
        self.camera_height = camera_height

    def detect(
        self,
        depth_data: np.ndarray,
        bbox: Tuple[int, int, int, int],
        estimated_distance_m: float,
    ) -> Tuple[bool, float, float, float, float, float, float, float, float, float]:
        """
        Returns: (valid, cx, cy, left, right, top, bottom, w, h, min_clearance_m)
        """
        h, w = depth_data.shape
        x1, y1, x2, y2 = bbox

        x1_c = max(0, int(x1))
        y1_c = max(0, int(y1))
        x2_c = min(w, int(x2))
        y2_c = min(h, int(y2))

        if x2_c - x1_c < 10 or y2_c - y1_c < 10:
            return (False, 0, 0, 0, 0, 0, 0, 0, 0, 99.0)

        roi = depth_data[y1_c:y2_c, x1_c:x2_c]
        roi_h, roi_w = roi.shape

        scale = 1.0
        if roi_w > 120:
            scale = 120.0 / roi_w
            new_w = 120
            new_h = max(8, int(roi_h * scale))
            roi_small = self._resize_depth(roi, new_w, new_h)
        else:
            roi_small = roi
            new_h, new_w = roi_small.shape

        far_mask = roi_small > self.depth_threshold_mm

        left_edges = []
        right_edges = []
        for row in range(new_h):
            far_pixels = np.where(far_mask[row, :])[0]
            if len(far_pixels) >= new_w * self.min_opening_ratio:
                left_edges.append(far_pixels[0])
                right_edges.append(far_pixels[-1])

        top_edges = []
        bottom_edges = []
        for col in range(new_w):
            far_pixels = np.where(far_mask[:, col])[0]
            if len(far_pixels) >= new_h * self.min_opening_ratio:
                top_edges.append(far_pixels[0])
                bottom_edges.append(far_pixels[-1])

        min_consensus = max(5, int(new_h * self.edge_consensus_ratio))
        if len(left_edges) < min_consensus or len(top_edges) < min_consensus:
            return (False, 0, 0, 0, 0, 0, 0, 0, 0, 99.0)

        left_s = float(np.median(left_edges))
        right_s = float(np.median(right_edges))
        top_s = float(np.median(top_edges))
        bottom_s = float(np.median(bottom_edges))

        left_edge = x1_c + left_s / scale
        right_edge = x1_c + right_s / scale
        top_edge = y1_c + top_s / scale
        bottom_edge = y1_c + bottom_s / scale

        center_cx = (left_edge + right_edge) / 2.0
        center_cy = (top_edge + bottom_edge) / 2.0
        opening_w = right_edge - left_edge
        opening_h = bottom_edge - top_edge

        dist = max(0.5, estimated_distance_m)
        cl = (center_cx - left_edge) * dist / self.fx
        cr = (right_edge - center_cx) * dist / self.fx
        ct = (center_cy - top_edge) * dist / self.fy
        cb = (bottom_edge - center_cy) * dist / self.fy
        min_cl = min(cl, cr, ct, cb)

        return (True, center_cx, center_cy, left_edge, right_edge,
                top_edge, bottom_edge, opening_w, opening_h, min_cl)

    @staticmethod
    def _resize_depth(roi: np.ndarray, new_w: int, new_h: int) -> np.ndarray:
        """
        Block-median downscale (mengabaikan 0 = invalid), TERVEKTORISASI.

        Implementasi lama memakai double loop Python (~8000 np.median per
        frame ≈ 300ms memegang GIL) — membuat thread monitor memblokir
        callback Hailo dan main loop. Versi ini mereplikasi PERSIS batas
        patch lama (floor(arange * dim / new_dim), ukuran patch non-uniform)
        dan median-nya, via lexsort + group-median. ~100x lebih cepat.
        """
        h, w = roi.shape
        if new_w > w or new_h > h:
            return roi

        # Label blok per piksel — ekuivalen dengan patch
        # roi[iy[ny]:iy[ny+1], ix[nx]:ix[nx+1]] pada implementasi lama.
        label_y = ((np.arange(h) + 1) * new_h - 1) // h
        label_x = ((np.arange(w) + 1) * new_w - 1) // w
        block_id = (label_y[:, None] * new_w + label_x[None, :]).ravel()
        values = roi.ravel()

        # Sort per (block, value) → tiap blok contiguous & ascending
        order = np.lexsort((values, block_id))
        sorted_vals = values[order].astype(np.uint32)

        n_blocks = new_h * new_w
        counts = np.bincount(block_id, minlength=n_blocks)
        starts = np.zeros(n_blocks, dtype=np.int64)
        np.cumsum(counts[:-1], out=starts[1:])
        zeros = np.add.reduceat(
            (sorted_vals == 0).astype(np.int64), starts
        )
        valid = counts - zeros

        # Median nilai valid (0 mengumpul di awal tiap blok). Jumlah valid
        # GENAP → rata-rata dua nilai tengah: (lo + hi) // 2 identik dengan
        # int(np.median) untuk nilai non-negatif.
        idx_hi = starts + zeros + valid // 2
        idx_lo = idx_hi - (1 - valid % 2)
        idx_hi = np.clip(idx_hi, 0, sorted_vals.size - 1)
        idx_lo = np.clip(idx_lo, 0, sorted_vals.size - 1)
        out = ((sorted_vals[idx_lo] + sorted_vals[idx_hi]) // 2).astype(roi.dtype)
        return np.where(valid > 0, out, 0).reshape(new_h, new_w)


class DepthSafetyMonitor:
    """
    D435i stereo depth anti-collision monitor.

    Standalone mode (creates own pipeline):
        monitor = DepthSafetyMonitor()
        monitor.start()                 # creates pipeline + thread

    External mode (receives frames from shared pipeline):
        monitor = DepthSafetyMonitor()
        monitor.start(external=True)    # thread only, no pipeline
        monitor.push_frame(depth_data, timestamp)  # called from main thread

    Read state (both modes):
        state = monitor.get_state()
    """

    def __init__(
        self,
        depth_width: int = cfg.DEPTH_WIDTH,
        depth_height: int = cfg.DEPTH_HEIGHT,
        depth_fps: int = cfg.DEPTH_FPS,
        forward_roi_size: int = cfg.DEPTH_FORWARD_ROI_SIZE,
        min_clearance_m: float = cfg.DEPTH_MIN_CLEARANCE_M,
        emergency_m: float = cfg.DEPTH_FORWARD_EMERGENCY_M,
        gate_center_clear_m: float = cfg.DEPTH_GATE_CENTER_CLEAR_M,
        edge_ratio: float = cfg.DEPTH_EDGE_CLEARANCE_RATIO,
        asymmetry_m: float = cfg.DEPTH_ASYMMETRY_THRESHOLD_M,
        temporal_window: int = cfg.DEPTH_TEMPORAL_WINDOW,
        noise_warn: float = cfg.DEPTH_NOISE_WARN_THRESHOLD,
        max_depth: float = cfg.DEPTH_MEDIAN_MAX_DEPTH,
        camera_width: int = cfg.CAMERA_WIDTH,
        camera_height: int = cfg.CAMERA_HEIGHT,
        aperture_every: int = 1,
        build_colormap: bool = True,
    ):
        self.depth_width = depth_width
        self.depth_height = depth_height
        self.depth_fps = depth_fps
        self.forward_roi_size = forward_roi_size
        self.min_clearance_m = min_clearance_m
        self.emergency_m = emergency_m
        self.gate_center_clear_m = gate_center_clear_m
        self.edge_ratio = edge_ratio
        self.asymmetry_m = asymmetry_m
        self.temporal_window = temporal_window
        self.noise_warn = noise_warn
        self.max_depth = max_depth
        self.camera_width = camera_width
        self.camera_height = camera_height
        # Throttle aperture detection (python-loop, paling berat) — hasil
        # terakhir di-cache dan dipakai ulang di frame di antaranya.
        self.aperture_every = max(1, int(aperture_every))
        self._aperture_counter = 0
        self._aperture_miss = 0
        self._last_aperture: Tuple = (
            False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99.0
        )
        # Colormap hanya untuk HUD/viewer — skip jika tidak dipakai (hemat CPU)
        self.build_colormap = build_colormap

        self._pipeline: Optional["rs.pipeline"] = None
        self._align: Optional["rs.align"] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._external_mode = False
        self._frame_queue: Optional[queue.Queue] = None
        self._lock = threading.Lock()
        self._state = SafetyState()

        self._gate_bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)
        self._gate_bbox_valid = False
        self._bbox_lock = threading.Lock()

        self._forward_history: deque = deque(maxlen=temporal_window)
        self._center_history: deque = deque(maxlen=temporal_window)
        self._left_history: deque = deque(maxlen=temporal_window)
        self._right_history: deque = deque(maxlen=temporal_window)
        self._top_history: deque = deque(maxlen=temporal_window)
        self._bottom_history: deque = deque(maxlen=temporal_window)

        self._depth_frame: Optional[np.ndarray] = None
        self._depth_colormap: Optional[np.ndarray] = None
        self._colormap_lock = threading.Lock()

        self._fps = 0.0
        self._prev_time = 0.0

        self._aperture_detector = ApertureDetector(
            camera_width=depth_width,
            camera_height=depth_height,
            fx=cfg.FX * depth_width / camera_width,
            fy=cfg.FY * depth_height / camera_height,
        )

    # ── Lifecycle ───────────────────────────────────────────────────────────

    def start(self, external: bool = False) -> bool:
        """Start depth processing.

        Args:
            external: If True, runs in push_frame mode (no own pipeline).
                      Frames must be fed via push_frame().
                      If False (standalone), creates own RealSense pipeline.

        Returns:
            True if started successfully.
        """
        if self._running:
            return True

        self._external_mode = external

        if external:
            return self._start_external()
        else:
            return self._start_standalone()

    def _start_external(self) -> bool:
        """Start in external (push_frame) mode — no pipeline, just processing thread."""
        if not HAS_REALSENSE:
            logger.error("DepthSafety: pyrealsense2 not installed")
            return False

        self._frame_queue = queue.Queue(maxsize=2)
        self._running = True
        self._thread = threading.Thread(
            target=self._run_external, name="DepthSafety", daemon=True
        )
        self._thread.start()
        with self._lock:
            self._state.depth_available = True
        logger.info("DepthSafetyMonitor: started (external/push_frame mode)")
        return True

    def _start_standalone(self) -> bool:
        """Start in standalone mode — creates own RealSense pipeline."""
        if not HAS_REALSENSE:
            logger.error("DepthSafety: pyrealsense2 not installed")
            return False

        try:
            self._pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(
                rs.stream.depth,
                self.depth_width, self.depth_height,
                rs.format.z16, self.depth_fps,
            )
            config.enable_stream(
                rs.stream.color,
                self.camera_width, self.camera_height,
                rs.format.bgr8, self.depth_fps,
            )
            profile = self._pipeline.start(config)

            depth_sensor = profile.get_device().first_depth_sensor()
            if depth_sensor.supports(rs.option.visual_preset):
                try:
                    depth_sensor.set_option(rs.option.visual_preset, 4.0)
                except RuntimeError:
                    try:
                        depth_sensor.set_option(rs.option.visual_preset, 1.0)
                    except RuntimeError:
                        pass

            self._align = rs.align(rs.stream.color)
            self._running = True
            self._thread = threading.Thread(
                target=self._run_standalone, name="DepthSafety", daemon=True
            )
            self._thread.start()

            with self._lock:
                self._state.depth_available = True

            logger.info(
                "DepthSafetyMonitor: started (standalone, preset=%s)",
                depth_sensor.get_option(rs.option.visual_preset)
                if depth_sensor.supports(rs.option.visual_preset)
                else "default",
            )
            return True

        except Exception as e:
            logger.error("DepthSafetyMonitor: failed to start — %s", e)
            self._running = False
            with self._lock:
                self._state.depth_available = False
            return False

    def stop(self):
        """Stop depth processing and join background thread."""
        self._running = False
        if self._external_mode and self._frame_queue is not None:
            try:
                self._frame_queue.put_nowait((None, 0.0))  # wake up thread
            except queue.Full:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._pipeline:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        with self._lock:
            self._state.depth_available = False
        logger.info("DepthSafetyMonitor: stopped")

    @property
    def is_running(self) -> bool:
        return self._running

    # ── Frame Input (external mode) ─────────────────────────────────────────

    def push_frame(self, depth_data: np.ndarray, timestamp: float):
        """
        Push a pre-aligned depth frame (16-bit uint, mm) for processing.
        Called from main thread. Non-blocking — drops oldest if queue full.

        Args:
            depth_data: numpy array (H, W) of uint16 depth in millimeters.
            timestamp: float seconds (time.perf_counter()).
        """
        if not self._external_mode or self._frame_queue is None:
            return
        if self._frame_queue.full():
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                pass
        try:
            self._frame_queue.put_nowait((depth_data, timestamp))
        except queue.Full:
            pass

    # ── Gate BBox ───────────────────────────────────────────────────────────

    def set_gate_bbox(self, x1: int, y1: int, x2: int, y2: int):
        """Set current gate bbox in depth frame coordinates (scaled from RGB)."""
        dw = self.depth_width
        dh = self.depth_height
        x1_c = max(0, min(dw - 1, x1))
        y1_c = max(0, min(dh - 1, y1))
        x2_c = max(x1_c + 1, min(dw, x2))
        y2_c = max(y1_c + 1, min(dh, y2))
        with self._bbox_lock:
            self._gate_bbox = (x1_c, y1_c, x2_c, y2_c)
            self._gate_bbox_valid = True

    def clear_gate_bbox(self):
        """Clear gate bbox (e.g. when lost)."""
        with self._bbox_lock:
            self._gate_bbox_valid = False

    # ── Thread-Safe State Access ────────────────────────────────────────────

    def get_state(self) -> SafetyState:
        """Return a copy of current safety state. Fast, single lock acquire."""
        with self._lock:
            s = self._state
            gc_copy = None
            if s.gate_clearance is not None:
                gc = s.gate_clearance
                gc_copy = ClearanceResult(
                    is_clear=gc.is_clear,
                    is_symmetric=gc.is_symmetric,
                    center_depth=gc.center_depth,
                    left_depth=gc.left_depth,
                    right_depth=gc.right_depth,
                    top_depth=gc.top_depth,
                    bottom_depth=gc.bottom_depth,
                    edge_ratio=gc.edge_ratio,
                    warning=gc.warning,
                )
            return SafetyState(
                forward_clear=s.forward_clear,
                forward_depth=s.forward_depth,
                gate_clearance=gc_copy,
                measured_distance=s.measured_distance,
                depth_available=s.depth_available,
                noise_level=s.noise_level,
                frame_timestamp=s.frame_timestamp,
                fps=s.fps,
                aperture_valid=s.aperture_valid,
                aperture_cx=s.aperture_cx,
                aperture_cy=s.aperture_cy,
                aperture_left=s.aperture_left,
                aperture_right=s.aperture_right,
                aperture_top=s.aperture_top,
                aperture_bottom=s.aperture_bottom,
                aperture_w=s.aperture_w,
                aperture_h=s.aperture_h,
                aperture_clearance_m=s.aperture_clearance_m,
            )

    def get_depth_colormap(self) -> Optional[np.ndarray]:
        """Get BGR colormap of latest depth frame for HUD overlay."""
        with self._colormap_lock:
            if self._depth_colormap is None:
                return None
            return self._depth_colormap.copy()

    def get_depth_frame(self) -> Optional[np.ndarray]:
        """Get raw aligned depth frame (16-bit uint, mm). For standalone viewer."""
        with self._colormap_lock:
            if self._depth_frame is None:
                return None
            return self._depth_frame.copy()

    # ── Background Threads ───────────────────────────────────────────────────

    def _run_external(self):
        """Background thread — pops frames from queue, processes them."""
        self._prev_time = time.perf_counter()
        while self._running:
            try:
                item = self._frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            depth_data, timestamp = item
            if depth_data is None:
                continue

            t_now = time.perf_counter()
            dt = t_now - self._prev_time
            self._prev_time = t_now
            if dt > 0:
                self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)

            with self._colormap_lock:
                self._depth_frame = depth_data

            self._process_frame(depth_data, timestamp)

    def _run_standalone(self):
        """Background thread — reads frames from own RealSense pipeline."""
        self._prev_time = time.perf_counter()
        consecutive_failures = 0

        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=500)
                if not frames:
                    consecutive_failures += 1
                    if consecutive_failures > 10:
                        logger.error("DepthSafety: 10 consecutive frame timeouts!")
                        self._set_depth_unavailable()
                    continue
                consecutive_failures = 0

                aligned = self._align.process(frames)
                depth_frame = aligned.get_depth_frame()
                if not depth_frame:
                    continue

                t_now = time.perf_counter()
                dt = t_now - self._prev_time
                self._prev_time = t_now
                if dt > 0:
                    self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)

                depth_data = np.asanyarray(depth_frame.get_data())
                with self._colormap_lock:
                    self._depth_frame = depth_data

                self._process_frame(depth_data, t_now)

            except RuntimeError as e:
                logger.error("DepthSafety: runtime error — %s", e)
                consecutive_failures += 1
                if consecutive_failures > 5:
                    self._set_depth_unavailable()
                    break
                time.sleep(0.1)
            except Exception as e:
                logger.exception("DepthSafety: unexpected error — %s", e)
                consecutive_failures += 1
                if consecutive_failures > 3:
                    break
                time.sleep(0.1)

    def _set_depth_unavailable(self):
        with self._lock:
            self._state.depth_available = False
            self._state.forward_clear = True
            self._state.gate_clearance = None

    def _process_frame(self, depth_data: np.ndarray, timestamp: float):
        """Process one depth frame: forward check + gate clearance + distance."""
        h, w = depth_data.shape

        cy = h // 2
        cx = w // 2
        half_roi = self.forward_roi_size // 2
        y1_f = max(0, cy - half_roi)
        y2_f = min(h, cy + half_roi)
        x1_f = max(0, cx - half_roi)
        x2_f = min(w, cx + half_roi)

        forward_roi = depth_data[y1_f:y2_f, x1_f:x2_f]
        forward_depth = self._median_valid(forward_roi)

        self._forward_history.append(forward_depth)
        fwd_temporal = float(np.median(list(self._forward_history)))

        forward_clear = fwd_temporal >= self.min_clearance_m
        forward_emergency = fwd_temporal < self.emergency_m

        valid = forward_roi[(forward_roi > 0) & (forward_roi < 15000)]
        noise = float(np.std(valid)) / 1000.0 if len(valid) > 10 else 0.0

        center_roi = depth_data[
            max(0, cy - 10):min(h, cy + 10),
            max(0, cx - 10):min(w, cx + 10),
        ]
        measured_dist = self._median_valid(center_roi)

        clearance = None
        gate_bbox = None
        with self._bbox_lock:
            gate_bbox = self._gate_bbox if self._gate_bbox_valid else None
        if gate_bbox is not None:
            clearance = self._check_gate_clearance(depth_data, gate_bbox)

        # ── Aperture detection (true gate opening dari depth edges) ──
        # Dithrottle: hanya tiap aperture_every frame (hasil di-cache) karena
        # _resize_depth memakai python loop — paling berat di pipeline ini.
        # Cache di-reset bila kondisi deteksi gagal berkepanjangan (anti-stale).
        if gate_bbox is not None and measured_dist < 99.0:
            self._aperture_counter += 1
            self._aperture_miss = 0
            if self._aperture_counter >= self.aperture_every:
                self._aperture_counter = 0
                self._last_aperture = self._aperture_detector.detect(
                    depth_data, gate_bbox, measured_dist
                )
        else:
            self._aperture_miss += 1
            if self._aperture_miss > self.aperture_every * 2:
                self._aperture_counter = 0
                self._last_aperture = (
                    False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99.0
                )

        ap_valid, ap_cx, ap_cy, ap_l, ap_r, ap_t, ap_b, ap_w, ap_h, ap_cl = (
            self._last_aperture
        )
        if ap_valid:
            # Scale back to RGB frame coordinates for HUD overlay
            sx = self.camera_width / self.depth_width
            sy = self.camera_height / self.depth_height
            ap_cx *= sx; ap_cy *= sy
            ap_l *= sx; ap_r *= sx
            ap_t *= sy; ap_b *= sy
            ap_w *= sx; ap_h *= sy

        with self._lock:
            st = self._state
            st.forward_clear = forward_clear and not forward_emergency
            st.forward_depth = fwd_temporal
            st.gate_clearance = clearance
            st.measured_distance = measured_dist
            st.noise_level = noise
            st.frame_timestamp = timestamp
            st.fps = self._fps
            st.depth_available = True
            st.aperture_valid = ap_valid
            st.aperture_cx = ap_cx
            st.aperture_cy = ap_cy
            st.aperture_left = ap_l
            st.aperture_right = ap_r
            st.aperture_top = ap_t
            st.aperture_bottom = ap_b
            st.aperture_w = ap_w
            st.aperture_h = ap_h
            st.aperture_clearance_m = ap_cl

        if self.build_colormap:
            self._build_colormap(
                depth_data, x1_f, y1_f, x2_f, y2_f, gate_bbox, forward_emergency
            )

    def _build_colormap(
        self,
        depth_data: np.ndarray,
        x1_f: int, y1_f: int, x2_f: int, y2_f: int,
        bbox: Optional[Tuple[int, int, int, int]],
        emergency: bool,
    ):
        """Build 3-channel BGR colormap of depth for HUD overlay."""
        if cv2 is None:
            with self._colormap_lock:
                self._depth_colormap = None
            return

        valid_mask = (depth_data > 0) & (depth_data < int(self.max_depth * 1000))
        depth_clipped = np.where(valid_mask, depth_data, 0).astype(np.float32)
        depth_norm = np.clip(depth_clipped / (self.max_depth * 1000.0), 0.0, 1.0)
        cmap = (depth_norm * 255).astype(np.uint8)
        colormap = cv2.applyColorMap(cmap, cv2.COLORMAP_JET)

        if emergency:
            overlay = np.zeros_like(colormap)
            overlay[:, :] = (0, 0, 255)
            colormap = cv2.addWeighted(colormap, 0.6, overlay, 0.4, 0)

        cv2.rectangle(
            colormap, (x1_f, y1_f), (x2_f, y2_f),
            (0, 255, 255) if not emergency else (0, 0, 255), 1,
        )

        if bbox is not None:
            bx1, by1, bx2, by2 = bbox
            edge_w = max(1, int((bx2 - bx1) * self.edge_ratio))
            edge_h = max(1, int((by2 - by1) * self.edge_ratio))
            cv2.rectangle(
                colormap, (bx1 + edge_w, by1 + edge_h),
                (bx2 - edge_w, by2 - edge_h), (0, 255, 0), 1,
            )

        with self._colormap_lock:
            self._depth_colormap = colormap

    # ── Gate Clearance Logic ────────────────────────────────────────────────

    def _check_gate_clearance(
        self, depth_data: np.ndarray, bbox: Tuple[int, int, int, int]
    ) -> ClearanceResult:
        h, w = depth_data.shape
        x1, y1, x2, y2 = bbox

        if x2 <= x1 or y2 <= y1:
            return ClearanceResult(warning="invalid bbox")

        bw = x2 - x1
        bh = y2 - y1
        edge_w = max(1, int(bw * self.edge_ratio))
        edge_h = max(1, int(bh * self.edge_ratio))

        x1_c = max(0, x1)
        y1_c = max(0, y1)
        x2_c = min(w, x2)
        y2_c = min(h, y2)

        if x2_c <= x1_c or y2_c <= y1_c:
            return ClearanceResult(warning="bbox outside frame")

        bbox_roi = depth_data[y1_c:y2_c, x1_c:x2_c]
        roi_h, roi_w = bbox_roi.shape
        ew = min(edge_w, roi_w // 3)
        eh = min(edge_h, roi_h // 3)
        cw = roi_w - 2 * ew
        ch = roi_h - 2 * eh

        if cw < 4 or ch < 4:
            return ClearanceResult(warning="bbox too small for clearance check")

        center = bbox_roi[eh:eh + ch, ew:ew + cw]
        left = bbox_roi[eh:eh + ch, :ew]
        right = bbox_roi[eh:eh + ch, roi_w - ew:]
        top = bbox_roi[:eh, ew:ew + cw]
        bottom = bbox_roi[roi_h - eh:, ew:ew + cw]

        center_m = self._median_valid(center)
        left_m = self._median_valid(left)
        right_m = self._median_valid(right)
        top_m = self._median_valid(top)
        bottom_m = self._median_valid(bottom)

        self._center_history.append(center_m)
        self._left_history.append(left_m)
        self._right_history.append(right_m)
        self._top_history.append(top_m)
        self._bottom_history.append(bottom_m)

        center_t = float(np.median(list(self._center_history)))
        left_t = float(np.median(list(self._left_history)))
        right_t = float(np.median(list(self._right_history)))
        top_t = float(np.median(list(self._top_history)))
        bottom_t = float(np.median(list(self._bottom_history)))

        is_clear = center_t >= self.gate_center_clear_m
        is_symmetric = abs(left_t - right_t) < self.asymmetry_m

        edge_ratio_val = (
            (left_t + right_t) / max(center_t * 2, 0.01)
            if center_t > 0 else 1.0
        )

        warning = ""
        if not is_clear:
            if center_t < self.emergency_m:
                warning = (
                    f"GATE BLOCKED! center={center_t:.1f}m "
                    f"< {self.emergency_m}m — ABORT PASS"
                )
            else:
                warning = (
                    f"Gate aperture unclear: center={center_t:.1f}m "
                    f"< {self.gate_center_clear_m}m — hold alignment"
                )
        elif not is_symmetric:
            offset_dir = "left" if left_t < right_t else "right"
            offset_m = abs(left_t - right_t)
            warning = (
                f"Gate asymmetry: {offset_dir} edge closer by {offset_m:.1f}m "
                f"(left={left_t:.1f}m right={right_t:.1f}m)"
            )

        return ClearanceResult(
            is_clear=is_clear,
            is_symmetric=is_symmetric,
            center_depth=center_t,
            left_depth=left_t,
            right_depth=right_t,
            top_depth=top_t,
            bottom_depth=bottom_t,
            edge_ratio=edge_ratio_val,
            warning=warning,
        )

    @staticmethod
    def _median_valid(data: np.ndarray) -> float:
        valid = data[(data > 0) & (data < 15000)]
        if len(valid) == 0:
            return 99.0
        return float(np.median(valid)) / 1000.0


# ── Module-level convenience ──────────────────────────────────────────────────


def create_monitor(
    enabled: bool = True,
    depth_width: int = cfg.DEPTH_WIDTH,
    depth_height: int = cfg.DEPTH_HEIGHT,
    depth_fps: int = cfg.DEPTH_FPS,
    external: bool = True,
    aperture_every: int = 1,
    build_colormap: bool = True,
) -> Optional[DepthSafetyMonitor]:
    """Factory: create and start a DepthSafetyMonitor.

    Args:
        enabled: If False, returns None immediately.
        external: If True, uses push_frame mode (no own pipeline).
                  If False, standalone mode (for depth_viewer.py).
        aperture_every: Run aperture detection every Nth processed frame
                        (cached in between). 1 = every frame (default).
        build_colormap: Build JET colormap each frame (for HUD/viewer).
                        Set False to save CPU when not displayed.

    Returns:
        DepthSafetyMonitor instance (already started), or None.
    """
    if not enabled:
        logger.info("DepthSafety: disabled by config")
        return None

    if not HAS_REALSENSE:
        logger.warning("DepthSafety: pyrealsense2 not available — skipping")
        return None

    try:
        monitor = DepthSafetyMonitor(
            depth_width=depth_width,
            depth_height=depth_height,
            depth_fps=depth_fps,
            aperture_every=aperture_every,
            build_colormap=build_colormap,
        )
        if monitor.start(external=external):
            return monitor
        logger.warning("DepthSafety: monitor failed to start — continuing without depth")
        return None
    except Exception as e:
        logger.warning("DepthSafety: %s — continuing without depth", e)
        return None
