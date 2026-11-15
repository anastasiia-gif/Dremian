"""
frame_source.py — where frames come from.

The whole demo talks to ONE interface: get_frame() returns a 2-D uint8 image
(grayscale thermal intensity, 0-255) or None when the stream ends.

Today that is a synthetic generator running on a laptop. On real hardware you
write ONE new class with the same get_frame() method (e.g. a V4L2 grab from the
Caddx capture chain) and the rest of the pipeline is unchanged. That single
seam is the point: the latency you measure for everything *after* get_frame()
is real algorithm latency; only get_frame() itself changes on hardware.
"""

import time
import numpy as np


class SyntheticThermalSource:
    """
    Generates a cold, noisy 'sky' background with a small hot point target
    that crosses the frame, plus an optional slower warm decoy ('bird').

    This stands in for the Caddx IRC-640CA feed for a laptop demo. It is NOT
    a model of the real camera-capture latency — see capture_is_real below.
    """

    capture_is_real = False  # honest flag the demo prints in its report

    def __init__(self, width=640, height=512, n_frames=300,
                 bg_noise=0.02, target_intensity=0.85, target_radius_px=2.0,
                 add_decoy=True, seed=42):
        self.w = width
        self.h = height
        self.n_frames = n_frames
        self.bg_noise = bg_noise
        self.target_intensity = target_intensity
        self.target_radius = target_radius_px
        self.add_decoy = add_decoy
        self.rng = np.random.default_rng(seed)
        self.i = 0

        # Target crosses left->right, slight vertical drift (a ~3 deg/s crosser).
        self.t_x0, self.t_y0 = 0.12 * width, 0.45 * height
        self.t_vx = 0.72 * width / n_frames   # px per frame
        self.t_vy = 0.10 * height / n_frames
        # Decoy: slower, lower, wanders (a bird ~ <1 deg/s).
        self.d_x0, self.d_y0 = 0.30 * width, 0.72 * height
        self.d_vx = 0.18 * width / n_frames

        # Ground-truth target position per frame, for scoring accuracy later.
        self.truth = []

    def _stamp_blob(self, img, cx, cy, intensity, radius):
        """Add a soft Gaussian hot spot at (cx, cy)."""
        x0 = max(0, int(cx - 4 * radius)); x1 = min(self.w, int(cx + 4 * radius) + 1)
        y0 = max(0, int(cy - 4 * radius)); y1 = min(self.h, int(cy + 4 * radius) + 1)
        if x1 <= x0 or y1 <= y0:
            return
        ys, xs = np.mgrid[y0:y1, x0:x1]
        g = np.exp(-(((xs - cx) ** 2 + (ys - cy) ** 2) / (2 * radius ** 2)))
        img[y0:y1, x0:x1] += (intensity * g).astype(np.float32)

    def get_frame(self):
        """Return (frame_uint8, truth_xy) or (None, None) at end of stream."""
        if self.i >= self.n_frames:
            return None, None
        i = self.i

        # Cold sky background + sensor noise.
        frame = self.rng.normal(0.06, self.bg_noise, (self.h, self.w)).astype(np.float32)

        # Hot target.
        tx = self.t_x0 + self.t_vx * i
        ty = self.t_y0 + self.t_vy * i
        self._stamp_blob(frame, tx, ty, self.target_intensity, self.target_radius)

        # Optional warm decoy.
        if self.add_decoy:
            dx = self.d_x0 + self.d_vx * i + 6 * np.sin(i * 0.15)
            dy = self.d_y0 + 5 * np.cos(i * 0.2)
            self._stamp_blob(frame, dx, dy, 0.45, self.target_radius * 1.3)

        frame = np.clip(frame, 0.0, 1.0)
        out = (frame * 255).astype(np.uint8)
        self.truth.append((tx, ty))
        self.i += 1
        return out, (tx, ty)


# ---------------------------------------------------------------------------
# Hardware seam (stub). On the Pi + capture chip you implement this and the
# rest of the demo runs unchanged. Left here so the structure is obvious.
# ---------------------------------------------------------------------------
class V4L2ThermalSource:
    """Placeholder for the real Caddx -> ADV7280A -> V4L2 capture path."""
    capture_is_real = True

    def __init__(self, device="/dev/video0", width=640, height=512):
        raise NotImplementedError(
            "Real capture not wired up in the laptop demo. On hardware, open the "
            "V4L2 device here and return a grayscale uint8 frame from get_frame()."
        )

    def get_frame(self):
        raise NotImplementedError
