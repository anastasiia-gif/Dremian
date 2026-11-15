"""
tracker.py — constant-velocity Kalman filter + proportional-navigation guidance.

KalmanTracker smooths the noisy detector centroid and coasts through short
dropouts (when detect() returns None). State = [x, y, vx, vy].

PNGuidance turns the tracked target's line-of-sight rate into a turn-rate
command, the same a_cmd = N * |Vc| * dλ/dt law as in the report (Eq 4.1).
Both are pure numpy so they run identically on laptop and Pi.
"""

import numpy as np


class KalmanTracker:
    def __init__(self, process_var=50.0, measure_var=4.0, dt=1.0):
        self.dt = dt
        self.q = process_var
        self.r = measure_var
        self.x = None          # state vector
        self.P = None          # covariance
        self.initialised = False

    def _init(self, z):
        self.x = np.array([z[0], z[1], 0.0, 0.0], dtype=np.float64)
        self.P = np.eye(4) * 500.0
        self.initialised = True

    def update(self, z):
        """z: (x, y) measurement or None (dropout -> predict only). Returns (x,y,vx,vy)."""
        if not self.initialised:
            if z is None:
                return None
            self._init(z)
            return tuple(self.x)

        dt = self.dt
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=np.float64)
        Q = np.eye(4) * self.q
        # Predict
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        # Update (only if we have a measurement)
        if z is not None:
            H = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0]], dtype=np.float64)
            R = np.eye(2) * self.r
            y = np.array(z, dtype=np.float64) - H @ self.x
            S = H @ self.P @ H.T + R
            K = self.P @ H.T @ np.linalg.inv(S)
            self.x = self.x + K @ y
            self.P = (np.eye(4) - K @ H) @ self.P
        return tuple(self.x)


class PNGuidance:
    """Proportional navigation: a_cmd = N * |Vc| * dλ/dt, clamped to max turn rate."""

    def __init__(self, N=4.0, fov_deg=50.0, frame_w=640,
                 max_turn_rate_dps=50.0, closing_speed_mps=107.0):
        self.N = N
        self.deg_per_px = fov_deg / frame_w        # angular scale
        self.max_turn = max_turn_rate_dps
        self.vc = closing_speed_mps
        self.prev_lambda = None

    def command(self, target_x, frame_w=640):
        """Return commanded turn rate (deg/s) from target bearing offset."""
        # Line-of-sight angle relative to boresight (deg).
        lam = (target_x - frame_w / 2.0) * self.deg_per_px
        if self.prev_lambda is None:
            self.prev_lambda = lam
            return 0.0
        dlam = lam - self.prev_lambda          # deg per frame ~ LOS rate
        self.prev_lambda = lam
        cmd = self.N * self.vc * np.radians(dlam)
        cmd_dps = np.degrees(cmd / max(self.vc, 1e-6))  # back to deg/s scale
        # Simpler, transparent form: command proportional to LOS rate.
        cmd_dps = self.N * dlam
        return float(np.clip(cmd_dps, -self.max_turn, self.max_turn))
