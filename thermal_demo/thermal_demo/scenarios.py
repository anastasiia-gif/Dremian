"""
scenarios.py — a small library of synthetic thermal clips.

Each scenario is a frame source with the SAME get_frame() interface as
SyntheticThermalSource, so the detector / tracker / web UI / latency harness
all run on any of them unchanged.

Scenarios:
  crossing    - small hot point crosses the frame (the standard detection case)
  approach    - target grows from a 2-px dot into a resolved delta/triangle
                shape as the interceptor closes in (terminal phase)
  decoy       - a real target plus a warm 'bird' decoy that must be rejected
  multi       - two hot objects; the faster crosser is the true target

Pick one with --scenario on run_demo.py / server.py.
"""

import numpy as np


def _soft_blob(img, cx, cy, intensity, radius):
    """Add a soft Gaussian hot spot (a point/unresolved target)."""
    h, w = img.shape
    x0 = max(0, int(cx - 4 * radius)); x1 = min(w, int(cx + 4 * radius) + 1)
    y0 = max(0, int(cy - 4 * radius)); y1 = min(h, int(cy + 4 * radius) + 1)
    if x1 <= x0 or y1 <= y0:
        return
    ys, xs = np.mgrid[y0:y1, x0:x1]
    g = np.exp(-(((xs - cx) ** 2 + (ys - cy) ** 2) / (2 * radius ** 2)))
    img[y0:y1, x0:x1] += (intensity * g).astype(np.float32)


def _delta_shape(img, cx, cy, size, intensity, angle=0.0):
    """
    Stamp a filled triangular (delta-wing) silhouette centred at (cx, cy).
    Represents a Shahed-class airframe once it is close enough to resolve.
    `size` is the half-span in pixels; grows as the target approaches.
    """
    h, w = img.shape
    # Triangle vertices: nose forward (+x), two tail corners back.
    pts = np.array([
        [ size * 1.4, 0.0],          # nose
        [-size * 0.8,  size],        # right tail
        [-size * 0.8, -size],        # left tail
    ])
    ca, sa = np.cos(angle), np.sin(angle)
    R = np.array([[ca, -sa], [sa, ca]])
    pts = pts @ R.T + np.array([cx, cy])

    # Rasterise the triangle by barycentric test over its bounding box.
    minx = max(0, int(pts[:, 0].min())); maxx = min(w, int(pts[:, 0].max()) + 1)
    miny = max(0, int(pts[:, 1].min())); maxy = min(h, int(pts[:, 1].max()) + 1)
    if maxx <= minx or maxy <= miny:
        return
    ys, xs = np.mgrid[miny:maxy, minx:maxx]
    p = np.stack([xs.ravel(), ys.ravel()], axis=1).astype(np.float32)
    a, b, c = pts
    v0, v1, v2 = b - a, c - a, p - a
    d00 = v0 @ v0; d01 = v0 @ v1; d11 = v1 @ v1
    d20 = v2 @ v0; d21 = v2 @ v1
    denom = d00 * d11 - d01 * d01 + 1e-9
    vv = (d11 * d20 - d01 * d21) / denom
    ww = (d00 * d21 - d01 * d20) / denom
    uu = 1.0 - vv - ww
    inside = (uu >= 0) & (vv >= 0) & (ww >= 0)
    mask = inside.reshape(ys.shape)
    patch = img[miny:maxy, minx:maxx]
    patch[mask] = np.maximum(patch[mask], intensity)
    # soft edge
    _soft_blob(img, cx, cy, intensity * 0.5, max(size * 0.6, 1.0))


class BaseScenario:
    capture_is_real = False

    def __init__(self, width=640, height=512, n_frames=300, seed=42, bg_noise=0.02):
        self.w = width; self.h = height; self.n_frames = n_frames
        self.bg_noise = bg_noise
        self.rng = np.random.default_rng(seed)
        self.i = 0
        self.truth = []

    def _bg(self):
        return self.rng.normal(0.06, self.bg_noise, (self.h, self.w)).astype(np.float32)

    def _finish(self, frame, tx, ty):
        frame = np.clip(frame, 0.0, 1.0)
        out = (frame * 255).astype(np.uint8)
        self.truth.append((tx, ty))
        self.i += 1
        return out, (tx, ty)


class CrossingScenario(BaseScenario):
    """Small hot point crosses left->right (standard detection case)."""
    name = "crossing"

    def get_frame(self):
        if self.i >= self.n_frames:
            return None, None
        f = self._bg()
        tx = 0.12 * self.w + (0.72 * self.w / self.n_frames) * self.i
        ty = 0.45 * self.h + (0.10 * self.h / self.n_frames) * self.i
        _soft_blob(f, tx, ty, 0.85, 2.0)
        return self._finish(f, tx, ty)


class ApproachScenario(BaseScenario):
    """
    Terminal approach: target starts as a 2-px point far away and grows into a
    resolved delta/triangle silhouette as range closes. Demonstrates that the
    detector locks the unresolved point early and stays locked as the shape
    resolves — the realistic intercept geometry.
    """
    name = "approach"

    def get_frame(self):
        if self.i >= self.n_frames:
            return None, None
        f = self._bg()
        frac = self.i / max(self.n_frames - 1, 1)        # 0 -> 1 as it approaches
        # comes in toward centre, drifting slightly
        tx = self.w * (0.30 + 0.20 * frac)
        ty = self.h * (0.40 + 0.10 * frac)
        if frac < 0.45:
            # far: unresolved hot point
            radius = 2.0 + 2.0 * frac
            _soft_blob(f, tx, ty, 0.9, radius)
            truth_x, truth_y = tx, ty
        else:
            # near: resolved triangle, grows large; gentle roll for realism
            size = 4.0 + 60.0 * (frac - 0.45) / 0.55
            angle = 0.25 * np.sin(self.i * 0.05)
            _delta_shape(f, tx, ty, size, 0.95, angle=angle)
            # Ground truth = centroid of the hot (target) pixels, which is what a
            # centroid-based detector reports. This keeps the accuracy metric
            # honest for an extended (resolved) target rather than comparing to
            # an arbitrary geometric point.
            hot = f > 0.5
            if hot.sum() > 0:
                ys, xs = np.nonzero(hot)
                truth_x, truth_y = float(xs.mean()), float(ys.mean())
            else:
                truth_x, truth_y = tx, ty
        frame = np.clip(f, 0.0, 1.0)
        out = (frame * 255).astype(np.uint8)
        self.truth.append((truth_x, truth_y))
        self.i += 1
        return out, (truth_x, truth_y)


class DecoyScenario(BaseScenario):
    """A true crossing target plus a warm slow decoy ('bird') to be rejected."""
    name = "decoy"

    def get_frame(self):
        if self.i >= self.n_frames:
            return None, None
        f = self._bg()
        tx = 0.12 * self.w + (0.72 * self.w / self.n_frames) * self.i
        ty = 0.45 * self.h + (0.10 * self.h / self.n_frames) * self.i
        _soft_blob(f, tx, ty, 0.85, 2.0)
        # decoy: slower, lower, wandering, slightly cooler
        dx = 0.30 * self.w + (0.18 * self.w / self.n_frames) * self.i + 6 * np.sin(self.i * 0.15)
        dy = 0.72 * self.h + 5 * np.cos(self.i * 0.2)
        _soft_blob(f, dx, dy, 0.45, 2.6)
        return self._finish(f, tx, ty)


class MultiScenario(BaseScenario):
    """Two hot objects; the faster crosser is the true target."""
    name = "multi"

    def get_frame(self):
        if self.i >= self.n_frames:
            return None, None
        f = self._bg()
        # true target: fast crosser
        tx = 0.10 * self.w + (0.80 * self.w / self.n_frames) * self.i
        ty = 0.35 * self.h + (0.05 * self.h / self.n_frames) * self.i
        _soft_blob(f, tx, ty, 0.85, 2.0)
        # slow second object (not the target)
        ox = 0.55 * self.w + (0.10 * self.w / self.n_frames) * self.i
        oy = 0.65 * self.h
        _soft_blob(f, ox, oy, 0.6, 2.2)
        return self._finish(f, tx, ty)


SCENARIOS = {
    "crossing": CrossingScenario,
    "approach": ApproachScenario,
    "decoy": DecoyScenario,
    "multi": MultiScenario,
}


def make_scenario(name, **kw):
    if name not in SCENARIOS:
        raise ValueError(f"unknown scenario '{name}'. choose from {list(SCENARIOS)}")
    return SCENARIOS[name](**kw)
