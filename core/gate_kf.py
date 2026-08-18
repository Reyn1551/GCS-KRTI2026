"""
KP2026 GateKF — Kalman Filter for gate tracking in pixel space.

Linear KF (not EKF) with 8-dim state: [cx, vcx, cy, vcy, w, vw, h, vh]
Measurement: [cx, cy, w, h] directly from YOLO bbox.

Features:
- Mahalanobis gating for outlier rejection
- Coasting (predict-only) when no detection
- est_distance computed post-filter from filtered width (non-linear)
"""

import logging
from typing import Optional, Tuple

import numpy as np

from config import CAMERA_WIDTH, GATE_REAL_WIDTH_M, FX

logger = logging.getLogger(__name__)


class GateKF:
    """Kalman Filter for gate tracking in pixel space."""

    # State dimensions
    NX = 8   # [cx, vcx, cy, vcy, w, vw, h, vh]
    NZ = 4   # [cx, cy, w, h] measurement

    def __init__(
        self,
        dt: float = 0.033,
        process_var_pos: float = 5.0,
        process_var_vel: float = 50.0,
        meas_var_default: float = 9.0,
        confidence_scale: float = 2.0,
        gate_threshold: float = 9.0,
        max_coast_time: float = 0.5,
        fx: float = FX,
        gate_real_width: float = GATE_REAL_WIDTH_M,
    ):
        """
        Args:
            dt: Time step (seconds). Updated dynamically if frames are irregular.
            process_var_pos: Process noise variance for position (px^2).
            process_var_vel: Process noise variance for velocity (px^2/s^2).
            meas_var_default: Base measurement noise variance (px^2).
            confidence_scale: R multiplier when confidence is low. R *= 1 + confidence_scale * (1 - conf).
            gate_threshold: Mahalanobis distance threshold for outlier gating (chi2, 4 dof, p=0.99).
            max_coast_time: Max seconds to coast (predict-only) without fresh measurement.
            fx: Camera focal length x (pixels) for distance estimation.
            gate_real_width: Real gate width (meters) for distance estimation.
        """
        self.dt = dt
        self.fx = fx
        self.gate_real_width = gate_real_width

        # State vector and covariance
        self.x = np.zeros(self.NX, dtype=np.float64)
        self.P = np.eye(self.NX, dtype=np.float64)

        # State transition matrix F (constant velocity per coordinate)
        self.F = self._build_F(self.dt)

        # Process noise Q
        self.Q = self._build_Q(process_var_pos, process_var_vel)

        # Measurement matrix H (select position components from state)
        self.H = np.zeros((self.NZ, self.NX), dtype=np.float64)
        self.H[0, 0] = 1.0  # cx
        self.H[1, 2] = 1.0  # cy
        self.H[2, 4] = 1.0  # w
        self.H[3, 6] = 1.0  # h

        # Measurement noise base
        self.R_base = meas_var_default * np.eye(self.NZ, dtype=np.float64)
        self.confidence_scale = confidence_scale

        # Outlier gating
        self.gate_threshold = gate_threshold  # chi2(4 dof, p=0.99) ≈ 13.28

        # Coasting
        self.max_coast_time = max_coast_time
        self._last_update_time: Optional[float] = None
        self._coasting = False

        # Tracking state
        self._initialized = False
        self._last_dt = dt

        logger.info(
            "GateKF: dt=%.3fs, Q_pos=%.1f, Q_vel=%.1f, R=%.1f, gate=%.1f, coast=%.2fs",
            dt, process_var_pos, process_var_vel, meas_var_default,
            gate_threshold, max_coast_time,
        )

    def _build_F(self, dt: float) -> np.ndarray:
        """Build state transition matrix for constant velocity model."""
        F = np.eye(self.NX, dtype=np.float64)
        # cx -> cx + vcx * dt
        F[0, 1] = dt
        # cy -> cy + vcy * dt
        F[2, 3] = dt
        # w -> w + vw * dt
        F[4, 5] = dt
        # h -> h + vh * dt
        F[6, 7] = dt
        return F

    def _build_Q(self, var_pos: float, var_vel: float) -> np.ndarray:
        """Build process noise covariance (constant velocity model)."""
        Q = np.zeros((self.NX, self.NX), dtype=np.float64)
        # Position process noise (integrated velocity noise)
        dt2 = self.dt ** 2
        dt3 = self.dt ** 3 / 3.0
        for i in [0, 2, 4, 6]:  # cx, cy, w, h positions
            Q[i, i] = var_pos * dt2 + var_vel * dt3
            Q[i, i + 1] = var_vel * dt2 / 2.0
            Q[i + 1, i] = var_vel * dt2 / 2.0
            Q[i + 1, i + 1] = var_vel * self.dt
        return Q

    def predict(self, timestamp: float) -> None:
        """
        Predict step. Updates state with constant-velocity model.

        Args:
            timestamp: Current frame timestamp (seconds).
        """
        if not self._initialized:
            self._last_update_time = timestamp
            return

        # Compute dt from last update
        if self._last_update_time is not None:
            dt = timestamp - self._last_update_time
            if dt > 0 and dt < 0.5:  # Sanity check
                self.dt = dt
                self.F = self._build_F(dt)
                self.Q = self._build_Q(
                    self.Q[0, 0] / max(self.dt ** 2, 1e-6),  # extract var_pos
                    self.Q[1, 1] / max(self.dt, 1e-6),       # extract var_vel
                )
            self._last_dt = dt

        # Predict state
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # Check coasting timeout
        if self._last_update_time is not None:
            time_since_update = timestamp - self._last_update_time
            if time_since_update > self.max_coast_time:
                if not self._coasting:
                    logger.warning("GateKF: coasting timeout (%.2fs), filter invalid", time_since_update)
                self._coasting = True

    def update(
        self,
        measurement: Tuple[float, float, float, float],
        confidence: float = 1.0,
        timestamp: Optional[float] = None,
    ) -> bool:
        """
        Update step with measurement.

        Args:
            measurement: (cx, cy, w, h) in pixels from YOLO bbox.
            confidence: Detection confidence [0, 1]. Affects R.
            timestamp: Frame timestamp for dt calculation.

        Returns:
            True if measurement accepted, False if rejected as outlier.
        """
        z = np.array(measurement, dtype=np.float64)

        # Initialize on first measurement
        if not self._initialized:
            self.x[0] = z[0]  # cx
            self.x[2] = z[1]  # cy
            self.x[4] = z[2]  # w
            self.x[6] = z[3]  # h
            # Initialize velocities to zero
            self.x[1] = 0.0   # vcx
            self.x[3] = 0.0   # vcy
            self.x[5] = 0.0   # vw
            self.x[7] = 0.0   # vh
            self.P = np.eye(self.NX, dtype=np.float64) * 100.0  # High initial uncertainty
            self._initialized = True
            self._last_update_time = timestamp
            self._coasting = False
            logger.debug("GateKF initialized: cx=%.1f cy=%.1f w=%.1f h=%.1f", *z)
            return True

        # Innovation
        z_pred = self.H @ self.x
        y = z - z_pred

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R_base

        # Mahalanobis distance for gating
        S_inv = np.linalg.inv(S)
        mahal = float(y @ S_inv @ y)

        # Outlier gating
        if mahal > self.gate_threshold:
            logger.debug(
                "GateKF: measurement REJECTED (mahal=%.2f > %.1f): "
                "z=(%.0f,%.0f,%.0f,%.0f) z_pred=(%.0f,%.0f,%.0f,%.0f)",
                mahal, self.gate_threshold, *z, *z_pred,
            )
            return False

        # Adaptive R based on confidence
        conf_factor = 1.0 + self.confidence_scale * (1.0 - max(0.0, min(1.0, confidence)))
        R = self.R_base * conf_factor

        # Enlarge R for width when bbox is large (close to gate → pixel noise more impactful)
        w_px = z[2]
        if w_px > CAMERA_WIDTH * 0.5:
            w_scale = 1.0 + 0.5 * ((w_px - CAMERA_WIDTH * 0.5) / (CAMERA_WIDTH * 0.5))
            R[2, 2] *= w_scale

        # Update state: K = P H^T S^{-1}
        S_inv = np.linalg.inv(S)
        K = self.P @ self.H.T @ S_inv

        self.x = self.x + K @ y
        self.P = (np.eye(self.NX) - K @ self.H) @ self.P

        # Ensure positive widths/heights
        self.x[4] = max(1.0, self.x[4])  # w
        self.x[6] = max(1.0, self.x[6])  # h

        self._last_update_time = timestamp
        self._coasting = False

        logger.debug(
            "GateKF update: mahal=%.2f conf=%.2f z=(%.0f,%.0f,%.0f,%.0f) "
            "state=(%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f)",
            mahal, confidence, *z, *self.x,
        )
        return True

    def reset(self) -> None:
        """Reset filter state."""
        self.x = np.zeros(self.NX, dtype=np.float64)
        self.P = np.eye(self.NX, dtype=np.float64)
        self._initialized = False
        self._last_update_time = None
        self._coasting = False
        logger.debug("GateKF reset")

    def is_valid(self) -> bool:
        """Check if filter has been initialized and is not coasting beyond timeout."""
        return self._initialized and not self._coasting

    def get_estimate(self) -> Tuple[float, float, float, float, float]:
        """
        Get filtered estimate.

        Returns:
            (cx, cy, w, h, est_distance) in pixels and meters.
        """
        cx = float(self.x[0])
        cy = float(self.x[2])
        w = float(self.x[4])
        h = float(self.x[6])

        # est_distance from filtered width (same formula as VisualGuidance.estimate_distance)
        if w > 1.0:
            est_dist = max(0.3, min(99.0, (self.gate_real_width * self.fx) / w))
        else:
            est_dist = 99.0

        return (cx, cy, w, h, est_dist)

    def get_velocities(self) -> Tuple[float, float, float, float]:
        """Get estimated velocities (pixels/second)."""
        return (float(self.x[1]), float(self.x[3]), float(self.x[5]), float(self.x[7]))

    @property
    def is_coasting(self) -> bool:
        """Whether filter is coasting (no fresh measurements)."""
        return self._coasting

    @property
    def age(self) -> float:
        """Time since last measurement update (seconds)."""
        if self._last_update_time is None:
            return 0.0
        return self._last_dt

    def __repr__(self) -> str:
        cx, cy, w, h, dist = self.get_estimate()
        vcx, vcy, vw, vh = self.get_velocities()
        return (
            f"GateKF(cx={cx:.0f}, cy={cy:.0f}, w={w:.0f}, h={h:.0f}, "
            f"dist={dist:.1f}m, v=({vcx:.0f},{vcy:.0f},{vw:.0f},{vh:.0f}), "
            f"coasting={self._coasting})"
        )
