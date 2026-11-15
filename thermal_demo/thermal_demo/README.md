# Thermal Detection Demo — live web UI, deployable, latency-measured

A self-contained demo of the Phase 2 thermal interceptor pipeline that you can
**watch live in a browser and measure** — built for the graduation defence.
No camera, no internet required: it generates synthetic thermal frames, runs
the full detect → track → guide chain, draws the detection on the video, and
reports real per-frame latency. Runs the same on a laptop and on a Raspberry Pi.

## 1. Watch it live (the main demo)

```bash
pip install -r requirements.txt     # numpy + opencv-python-headless (+ matplotlib)
python3 server.py                   # then open the URL it prints
```
Open **http://localhost:8000** in a browser. You'll see the thermal video, the
TARGET LOCK box tracking the hot target, the decoy being ignored, and live
latency / lock-rate read-outs. To run it on a Raspberry Pi and watch from your
laptop's browser, see **PI_SETUP.md**.

## 2. Measure latency to a file (for the report / a slide)

```bash
python3 run_demo.py                 # prints a live trace + writes latency_report.txt
python3 selftest.py                 # proves it's reproducible across 5 seeds
python3 plot_latency.py             # makes latency_timeline.png + latency_histogram.png
```

## 3. Multiple synthetic scenarios

There are four synthetic clips. Pick one with `--scenario`:

| Scenario   | What it shows | Lock rate |
|------------|---------------|-----------|
| `crossing` | a hot point crossing the frame (baseline)            | ~100% |
| `decoy`    | real target + a warm decoy that gets rejected        | ~100% |
| `approach` | target grows from a dot into a big delta/triangle as it closes (terminal geometry) | locks in the resolved phase |
| `multi`    | two hot objects; the faster crosser is the target    | ~100% |

```bash
python3 server.py --scenario approach     # watch the close-approach clip live
python3 run_demo.py --scenario decoy       # measure latency on a specific clip
python3 export_frames.py                   # save annotated PNGs from every scenario
```

`export_frames.py` writes ready-to-paste images into `figures/`. See
**REPORT_FIGURES.md** for exactly which image goes where in the report.

> Note on the `approach` clip: it is the *visual* demo of intercept geometry.
> The detector centroids on the brightest part of the resolved airframe, so the
> per-pixel "accuracy" number is not meaningful once the target is larger than
> the tracking gate — use `crossing`/`decoy` for the clean lock-rate metric and
> `approach` to show the dot-to-triangle story.

Outputs:
- `latency_report.txt` — the numbers to read out / put on a slide
- `latency_log.csv` — every frame, every stage (for auditing or your own plots)
- `latency_timeline.png`, `latency_histogram.png` — slide-ready figures

Only `numpy` is required to run the demo; `matplotlib` only for the plots.

## What it measures

Each frame is timed with a monotonic high-resolution clock at four points:

```
get_frame()  →  detect()  →  Kalman.update()  →  PN.command()
   capture        detect         track              guide
```

It reports each stage and two totals:
- **Algorithm latency** = detect + track + guide. *This is the figure that
  transfers to the Raspberry Pi* — it's pure computation on the same numpy code.
- **End-to-end latency** = capture + algorithm. On the laptop the capture stage
  is *synthetic frame generation*, not the real camera, so the end-to-end number
  is a laptop figure only.

## The honest story for the defence (say this out loud)

> "I built the detection pipeline as a deployable program and measured its
> latency directly, rather than estimating it. On my laptop the **algorithm**
> runs in about **4 ms per frame** (mean), reproducibly across random seeds,
> with the target locked ~100% of frames and the bird decoy rejected. That
> algorithm time is what carries to the Raspberry Pi, because it's the same
> code. What I have *not* yet measured is the real camera-capture delay
> (Caddx → ADV7280A → V4L2); that's a hardware measurement, and it's exactly
> Recommendation R1 in the report. So I can show measured algorithm latency
> today, and the end-to-end figure becomes real once the capture chain is on
> the bench."

This is defensible because it claims only what was measured and names the gap
explicitly. Do **not** present the laptop end-to-end number as the on-Pi number.

## How it maps to the report

| Component        | File          | Report section |
|------------------|---------------|----------------|
| Frame source     | `frame_source.py` | 4.3.3 sensor / Appendix I |
| Detector (5σ + motion gate) | `detector.py` | 4.3.3 algorithm, UR9a/SR2.7 target selection |
| Kalman tracker   | `tracker.py`  | SR2.5 |
| PN guidance      | `tracker.py`  | Eq 4.1 |
| Latency harness  | `run_demo.py` | 4.3.3 latency, Table 4.3.3.1, R1 |
| Live web UI       | `server.py`, `render.py` | demo / defence |

## Deploying on real hardware later

Write one class with a `get_frame()` method that returns a grayscale `uint8`
frame from the real capture device (a `V4L2ThermalSource` stub is in
`frame_source.py`). Swap it in for `SyntheticThermalSource` in `run_demo.py`.
Nothing else changes — and now the `capture` stage timing is real, giving you
the true end-to-end latency on the Pi.
