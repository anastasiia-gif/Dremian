"""
run_demo.py — deployable thermal-detection demo with honest latency measurement.

This is the thing you run at the defence. It:
  * generates synthetic thermal frames (no camera needed),
  * runs the full chain  capture -> detect -> track -> guide  per frame,
  * times EACH stage and the END-TO-END total per frame with a monotonic clock,
  * prints a live trace and a final latency report (mean / median / p95 / max),
  * writes latency_report.txt and latency_log.csv you can show or plot.

Honesty note baked into the output: on this laptop the 'capture' stage is
synthetic frame generation, NOT the real Caddx capture chain. The report says
so explicitly and reports algorithm latency (detect+track+guide) separately,
which is the figure that transfers to hardware. The end-to-end number here is a
laptop figure; the real on-Pi number must add the measured camera-capture delay.

Usage:
    python3 run_demo.py                 # default 300 frames
    python3 run_demo.py --frames 500    # more frames
    python3 run_demo.py --no-decoy      # drop the bird decoy
"""

import argparse
import time
import statistics as st

from frame_source import SyntheticThermalSource
from scenarios import make_scenario, SCENARIOS
from detector import ThermalDetector
from tracker import KalmanTracker, PNGuidance

# High-resolution monotonic timer (nanoseconds). Same call works on Pi.
clock = time.perf_counter_ns
NS = 1e6  # ns -> ms


def percentile(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    k = (len(sorted_vals) - 1) * (p / 100.0)
    f = int(k)
    c = min(f + 1, len(sorted_vals) - 1)
    if f == c:
        return sorted_vals[f]
    return sorted_vals[f] + (sorted_vals[c] - sorted_vals[f]) * (k - f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--no-decoy", action="store_true")
    ap.add_argument("--scenario", default=None, choices=list(SCENARIOS),
                    help="run a named scenario (crossing/decoy/approach/multi)")
    ap.add_argument("--quiet", action="store_true", help="suppress per-frame trace")
    args = ap.parse_args()

    if args.scenario:
        src = make_scenario(args.scenario, n_frames=args.frames, seed=42)
    else:
        src = SyntheticThermalSource(n_frames=args.frames, add_decoy=not args.no_decoy)
    det = ThermalDetector()
    kf = KalmanTracker()
    pn = PNGuidance()

    # Per-stage timing accumulators (milliseconds).
    t_capture, t_detect, t_track, t_guide, t_total = [], [], [], [], []
    locked_frames = 0
    on_target_frames = 0
    first_lock = None
    err_px = []

    print("=" * 64)
    print(" Thermal detection demo — live latency measurement")
    print(" capture(synthetic) -> detect -> track -> guide")
    print("=" * 64)

    frame_idx = 0
    while True:
        t0 = clock()
        frame, truth = src.get_frame()
        t1 = clock()
        if frame is None:
            break

        meas = det.detect(frame)
        t2 = clock()

        state = kf.update(meas)
        t3 = clock()

        cmd = 0.0
        if state is not None:
            cmd = pn.command(state[0], frame_w=frame.shape[1])
        t4 = clock()

        # Record stage latencies (ms).
        cap_ms = (t1 - t0) / NS
        det_ms = (t2 - t1) / NS
        trk_ms = (t3 - t2) / NS
        gui_ms = (t4 - t3) / NS
        tot_ms = (t4 - t0) / NS
        algo_ms = det_ms + trk_ms + gui_ms

        t_capture.append(cap_ms); t_detect.append(det_ms)
        t_track.append(trk_ms); t_guide.append(gui_ms); t_total.append(tot_ms)

        if state is not None and meas is not None:
            locked_frames += 1
            if first_lock is None:
                first_lock = frame_idx
            # accuracy vs ground truth
            if truth is not None:
                e = ((state[0] - truth[0]) ** 2 + (state[1] - truth[1]) ** 2) ** 0.5
                err_px.append(e)
                if e < 8.0:
                    on_target_frames += 1

        if not args.quiet and frame_idx % 25 == 0:
            tag = "LOCK" if (state is not None and meas is not None) else "----"
            print(f" f{frame_idx:>4} {tag}  algo={algo_ms:5.2f}ms  "
                  f"end2end={tot_ms:5.2f}ms  cmd={cmd:+6.1f} deg/s")
        frame_idx += 1

    # ---------------- report ----------------
    n = len(t_total)
    if n == 0:
        print("No frames processed.")
        return

    algo = [t_detect[i] + t_track[i] + t_guide[i] for i in range(n)]
    algo_s = sorted(algo)
    tot_s = sorted(t_total)

    def stat_block(name, vals):
        s = sorted(vals)
        return (f"  {name:<18} mean {st.mean(s):6.3f}  median {st.median(s):6.3f}  "
                f"p95 {percentile(s,95):6.3f}  max {max(s):6.3f}  (ms)")

    lines = []
    lines.append("=" * 64)
    lines.append(" THERMAL DETECTION — LATENCY REPORT")
    lines.append("=" * 64)
    lines.append(f" frames processed      : {n}")
    lines.append(f" platform              : laptop/PC (synthetic frame source)")
    lines.append(f" capture stage is real : {src.capture_is_real}")
    lines.append("")
    lines.append(" Per-stage latency:")
    lines.append(stat_block("capture(synth)", t_capture))
    lines.append(stat_block("detect", t_detect))
    lines.append(stat_block("track (Kalman)", t_track))
    lines.append(stat_block("guide (PN)", t_guide))
    lines.append("")
    lines.append(" Algorithm latency (detect+track+guide) -- TRANSFERS TO HARDWARE:")
    lines.append(stat_block("ALGORITHM", algo))
    lines.append("")
    lines.append(" End-to-end per-frame latency (laptop, incl. synthetic capture):")
    lines.append(stat_block("END-TO-END", t_total))
    lines.append("")
    if first_lock is not None:
        lines.append(f" first lock at frame   : {first_lock}")
    lines.append(f" frames locked         : {locked_frames}/{n}  ({100*locked_frames/n:.0f}%)")
    if err_px:
        lines.append(f" frames on-target(<8px): {on_target_frames}/{len(err_px)}  "
                     f"({100*on_target_frames/len(err_px):.0f}%)")
        lines.append(f" mean tracking error   : {st.mean(err_px):.2f} px")
    lines.append("")
    lines.append(" HONEST CAVEAT: 'capture(synth)' is synthetic frame generation,")
    lines.append(" not the real camera. The ALGORITHM figure is what carries to the")
    lines.append(" Raspberry Pi; the real end-to-end must add the measured Caddx ->")
    lines.append(" ADV7280A -> V4L2 capture-chain delay (a hardware measurement, R1).")
    lines.append("=" * 64)

    report = "\n".join(lines)
    print(report)
    with open("latency_report.txt", "w") as f:
        f.write(report + "\n")

    # CSV log for plotting/auditing every frame.
    with open("latency_log.csv", "w") as f:
        f.write("frame,capture_ms,detect_ms,track_ms,guide_ms,algorithm_ms,end_to_end_ms\n")
        for i in range(n):
            a = t_detect[i] + t_track[i] + t_guide[i]
            f.write(f"{i},{t_capture[i]:.4f},{t_detect[i]:.4f},{t_track[i]:.4f},"
                    f"{t_guide[i]:.4f},{a:.4f},{t_total[i]:.4f}\n")

    print("\nWrote latency_report.txt and latency_log.csv")


if __name__ == "__main__":
    main()
