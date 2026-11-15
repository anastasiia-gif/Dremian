"""
export_frames.py — save clean annotated sample frames from each scenario as PNGs.

These are the images to drop into the report (no screenshotting needed). For
each scenario it runs the pipeline and saves a few representative frames:
  - an early 'searching/lock' frame
  - a mid 'locked' frame
  - for the approach scenario, a 'resolved triangle' terminal frame

Output: figures/<scenario>_<tag>.png

Run:
    python export_frames.py
"""

import os
import numpy as np

from scenarios import make_scenario, SCENARIOS
from detector import ThermalDetector
from tracker import KalmanTracker, PNGuidance
import render

OUT = "figures"


def save_png(img_bgr, path):
    if render.HAVE_CV2:
        import cv2
        cv2.imwrite(path, img_bgr)
        return True
    return False


def run_and_capture(scenario, grab_frames):
    """Run a scenario; return {frame_idx: annotated_bgr} for requested indices."""
    src = make_scenario(scenario, n_frames=300, seed=42)
    det = ThermalDetector(); kf = KalmanTracker(); pn = PNGuidance()
    lock_count = 0; total = 0
    captured = {}
    while True:
        frame, truth = src.get_frame()
        if frame is None:
            break
        meas = det.detect(frame)
        state = kf.update(meas)
        cmd = pn.command(state[0], frame_w=frame.shape[1]) if state is not None else 0.0
        locked = (meas is not None and state is not None)
        total += 1
        if locked:
            lock_count += 1
        idx = src.i - 1
        if idx in grab_frames:
            img = render.annotate(frame, meas, state, locked, src.i,
                                  3.0, 8.0, cmd, lock_count, total)
            captured[idx] = img
    return captured


def main():
    if not render.HAVE_CV2:
        raise SystemExit("OpenCV needed for image export. pip install opencv-python-headless")
    os.makedirs(OUT, exist_ok=True)

    plan = {
        "crossing": {30: "early_lock", 150: "mid_lock", 270: "late_lock"},
        "decoy":    {40: "lock_decoy_rejected", 180: "tracking_with_decoy"},
        "approach": {60: "far_point", 200: "resolving", 285: "triangle_terminal"},
        "multi":    {60: "lock_true_target", 200: "tracking"},
    }

    saved = []
    for scen, frames in plan.items():
        caps = run_and_capture(scen, set(frames))
        for idx, tag in frames.items():
            if idx in caps:
                path = os.path.join(OUT, f"{scen}_{tag}.png")
                if save_png(caps[idx], path):
                    saved.append(path)
                    print("saved", path)
    print(f"\n{len(saved)} frames written to ./{OUT}/")
    print("These are ready to paste into the report (see REPORT_FIGURES.md).")


if __name__ == "__main__":
    main()
