"""
plot_latency.py — turn latency_log.csv into two figures for the defence:
  1. per-frame latency over time (algorithm vs end-to-end)
  2. histogram of algorithm latency with mean/p95 markers

Run AFTER run_demo.py. Saves latency_timeline.png and latency_histogram.png.
Falls back gracefully if matplotlib is missing (prints instructions).
"""

import csv

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    raise SystemExit("matplotlib not installed. Run: pip install matplotlib")


def load(path="latency_log.csv"):
    frames, algo, e2e = [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            frames.append(int(row["frame"]))
            algo.append(float(row["algorithm_ms"]))
            e2e.append(float(row["end_to_end_ms"]))
    return frames, algo, e2e


def main():
    frames, algo, e2e = load()
    mean_a = sum(algo) / len(algo)
    p95_a = sorted(algo)[int(0.95 * (len(algo) - 1))]

    # 1) timeline
    plt.figure(figsize=(9, 4))
    plt.plot(frames, e2e, lw=1.0, color="#c0742a", label="end-to-end (laptop)")
    plt.plot(frames, algo, lw=1.2, color="#2f6db0", label="algorithm (detect+track+guide)")
    plt.xlabel("frame"); plt.ylabel("latency (ms)")
    plt.title("Per-frame latency")
    plt.legend(); plt.grid(alpha=0.3); plt.tight_layout()
    plt.savefig("latency_timeline.png", dpi=140)
    plt.close()

    # 2) histogram of algorithm latency
    plt.figure(figsize=(7, 4))
    plt.hist(algo, bins=30, color="#2f6db0", alpha=0.8, edgecolor="white")
    plt.axvline(mean_a, color="#b03030", lw=2, label=f"mean {mean_a:.2f} ms")
    plt.axvline(p95_a, color="#3a8c43", lw=2, ls="--", label=f"p95 {p95_a:.2f} ms")
    plt.xlabel("algorithm latency (ms)"); plt.ylabel("frames")
    plt.title("Algorithm latency distribution (transfers to hardware)")
    plt.legend(); plt.grid(alpha=0.3); plt.tight_layout()
    plt.savefig("latency_histogram.png", dpi=140)
    plt.close()

    print("Wrote latency_timeline.png and latency_histogram.png")


if __name__ == "__main__":
    main()
