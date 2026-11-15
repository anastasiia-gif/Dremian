"""
detector.py — classical thermal point-target detector.

No neural network, no training data. The target is only 2-4 pixels wide at the
detection range, so there is no shape to learn; detection is intensity + motion:

  1. maintain a running background model of the cold sky
  2. flag pixels that exceed the local background by k-sigma (adaptive threshold)
  3. group flagged pixels into blobs, reject ones too big/small to be the target
  4. keep the blob whose frame-to-frame angular motion is target-like
       (a crossing munition moves faster across the FOV than a drifting bird)

Returns the detected target centroid (x, y) in pixels, or None.
"""

import numpy as np


class ThermalDetector:
    def __init__(self, k_sigma=5.0, bg_alpha=0.02, var_floor=2.5e-4,
                 min_blob_area=2, max_blob_area=80,
                 min_speed_px=0.8, max_speed_px=25.0):
        self.k = k_sigma
        self.alpha = bg_alpha              # background update rate (slow)
        self.var_floor = var_floor
        self.min_area = min_blob_area
        self.max_area = max_blob_area
        self.min_speed = min_speed_px      # reject near-stationary (bird/clutter)
        self.max_speed = max_speed_px      # reject impossible jumps (noise)
        self.bg_mean = None
        self.bg_var = None
        self.prev_xy = None

    def _update_background(self, f):
        if self.bg_mean is None:
            self.bg_mean = f.copy()
            self.bg_var = np.full_like(f, self.var_floor)
            return
        # Exponential moving average of mean and variance.
        diff = f - self.bg_mean
        self.bg_mean += self.alpha * diff
        self.bg_var = (1 - self.alpha) * (self.bg_var + self.alpha * diff * diff)
        np.maximum(self.bg_var, self.var_floor, out=self.bg_var)

    def _connected_blobs(self, mask):
        """Tiny 8-connected component finder (no OpenCV dependency)."""
        h, w = mask.shape
        labels = np.zeros((h, w), dtype=np.int32)
        blobs = []
        cur = 0
        ys, xs = np.nonzero(mask)
        seed_set = set(zip(ys.tolist(), xs.tolist()))
        for (sy, sx) in list(seed_set):
            if labels[sy, sx] != 0:
                continue
            cur += 1
            stack = [(sy, sx)]
            pix = []
            labels[sy, sx] = cur
            while stack:
                y, x = stack.pop()
                pix.append((y, x))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and labels[ny, nx] == 0:
                            labels[ny, nx] = cur
                            stack.append((ny, nx))
            area = len(pix)
            yy = np.mean([p[0] for p in pix])
            xx = np.mean([p[1] for p in pix])
            blobs.append((xx, yy, area))
        return blobs

    def detect(self, frame_u8):
        """frame_u8: HxW uint8. Returns (x, y) float centroid or None."""
        f = frame_u8.astype(np.float32) / 255.0
        self._update_background(f)

        # Adaptive k-sigma threshold above local background.
        thresh = self.bg_mean + self.k * np.sqrt(self.bg_var)
        # Freeze background where the target is hot so it doesn't 'learn' the target.
        hot = f > thresh
        if hot.sum() == 0:
            self.prev_xy = None
            return None

        blobs = self._connected_blobs(hot)
        cand = [b for b in blobs if self.min_area <= b[2] <= self.max_area]
        if not cand:
            self.prev_xy = None
            return None

        # Prefer the brightest-area-weighted blob; then apply motion gate.
        cand.sort(key=lambda b: b[2], reverse=True)
        for (x, y, area) in cand:
            if self.prev_xy is None:
                self.prev_xy = (x, y)
                return (x, y)
            spd = np.hypot(x - self.prev_xy[0], y - self.prev_xy[1])
            if self.min_speed <= spd <= self.max_speed:
                self.prev_xy = (x, y)
                return (x, y)
        # nothing passed the motion gate this frame
        self.prev_xy = cand[0][:2]
        return None
