"""
selftest.py — proves the pipeline is reproducible, not a lucky single run.

Runs the full chain across several random seeds and checks that on every seed
the detector locks the target, stays on it, rejects the decoy, and keeps
algorithm latency within a sane bound. Exits non-zero if any check fails, so it
can gate a demo ("the self-test passes on my machine") before the defence.
"""

import sys
import time
import statistics as st

from frame_source import SyntheticThermalSource
from detector import ThermalDetector
from tracker import KalmanTracker, PNGuidance

clock = time.perf_counter_ns
NS = 1e6


def run_one(seed, n_frames=300):
    src = SyntheticThermalSource(n_frames=n_frames, add_decoy=True, seed=seed)
    det = ThermalDetector()
    kf = KalmanTracker()
    pn = PNGuidance()
    locked = 0; on_t = 0; meas_frames = 0; algo = []
    while True:
        frame, truth = src.get_frame()
        if frame is None:
            break
        t1 = clock()
        meas = det.detect(frame)
        state = kf.update(meas)
        if state is not None:
            pn.command(state[0], frame_w=frame.shape[1])
        t2 = clock()
        algo.append((t2 - t1) / NS)
        if meas is not None and state is not None:
            locked += 1
            meas_frames += 1
            e = ((state[0] - truth[0]) ** 2 + (state[1] - truth[1]) ** 2) ** 0.5
            if e < 10.0:
                on_t += 1
    lock_pct = 100 * locked / n_frames
    ontgt_pct = 100 * on_t / max(meas_frames, 1)
    return lock_pct, ontgt_pct, st.mean(algo), max(algo)


def main():
    seeds = [1, 7, 42, 100, 2024]
    print("Self-test across seeds:", seeds)
    all_ok = True
    for s in seeds:
        lock_pct, ontgt_pct, mean_algo, max_algo = run_one(s)
        ok = lock_pct >= 80 and ontgt_pct >= 90 and mean_algo < 50.0
        all_ok &= ok
        flag = "PASS" if ok else "FAIL"
        print(f"  seed {s:>4}: lock {lock_pct:5.1f}%  on-target {ontgt_pct:5.1f}%  "
              f"algo mean {mean_algo:5.2f}ms max {max_algo:5.2f}ms  -> {flag}")
    print("OVERALL:", "PASS" if all_ok else "FAIL")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
