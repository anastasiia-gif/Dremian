# Which figures go where in the report

After you run `python export_frames.py`, you'll have a `figures/` folder with
the images below. Here's where each one belongs and a suggested caption. You can
copy-paste the images and captions yourself.

All of these belong in **Chapter 4.3 (Detection System)** and **Appendix I**,
which is where the thermal pipeline is described.

---

## 1. Standard detection — put in §4.3.3 (the detection-results paragraph)

**Image:** `crossing_mid_lock.png`
**Caption:** *Figure 4.3.3.x — Detection and lock on a crossing point target in
the synthetic thermal feed. The green box marks the Kalman-tracked target; the
HUD shows live algorithm latency and lock state.*

This is your "it works" baseline shot. One image is enough here.

---

## 2. Decoy rejection — put in §4.3.3, next to the target-selection text (UR9a / SR2.7)

**Image:** `decoy_lock_decoy_rejected.png`
**Caption:** *Figure 4.3.3.x — Target selection: the detector locks the fast
crossing target (green box) while the slower, cooler decoy (lower blob) is
rejected by the motion/area gate. This demonstrates the decoy-rejection
requirement (UR9a, SR2.7).*

Strong figure — it visually proves the target-selection requirement you added.

---

## 3. Terminal approach (dot → triangle) — put in §4.3.3 or Appendix I

Use these **two or three together** as a small sequence to show the intercept
geometry (target resolves as range closes):

| Image | Caption fragment |
|-------|------------------|
| `approach_far_point.png`        | (a) far range — unresolved hot point |
| `approach_resolving.png`        | (b) mid range — shape begins to resolve |
| `approach_triangle_terminal.png`| (c) terminal — resolved delta-wing silhouette |

**Caption:** *Figure 4.3.3.x — Terminal approach sequence. As range closes, the
target grows from an unresolved 2-px hot point (a) to a resolved delta-wing
thermal silhouette (c). The detector maintains lock throughout, confirming the
algorithm does not depend on target shape.*

This is the most informative figure for a thesis — it shows you thought about
the real engagement geometry, not just a static dot.

---

## 4. Latency evidence — put in §4.3.3 (latency) and reference Table 4.3.3.1

These come from `python run_demo.py` then `python plot_latency.py`:

| Image | Where | Caption |
|-------|-------|---------|
| `latency_histogram.png` | §4.3.3 latency paragraph | *Figure 4.3.3.x — Distribution of measured algorithm latency over 300 frames (mean and 95th percentile marked).* |
| `latency_timeline.png`  | Appendix I (supporting) | *Figure I.x — Per-frame algorithm and end-to-end latency.* |

Also paste the key numbers from `latency_report.txt` into the latency paragraph,
e.g. "the algorithm processed each frame in X ms on average (p95 Y ms), measured
over 300 frames." Use **your own measured number** from your laptop, not mine.

---

## What to SAY in the text (honesty point that protects you)

In the latency paragraph, keep the distinction explicit:

> The **algorithm** latency (detection + tracking + guidance) was measured
> directly and is the figure that transfers to the Raspberry Pi, since it is the
> same code. The end-to-end figure measured on the laptop includes a synthetic
> frame source rather than the real camera; the true camera-capture delay
> (Caddx → ADV7280A → V4L2) remains to be measured on hardware and is captured
> as Recommendation R1.

This way every figure and number is defensible: you measured what you measured,
and you named the gap.

---

## Quick recap of commands

```
python export_frames.py     # makes the figures/ folder (detection images)
python run_demo.py          # writes latency_report.txt + latency_log.csv
python plot_latency.py      # makes the two latency PNGs
python server.py --scenario approach   # live demo of the approach clip
```

Scenario options for the live demo: `crossing`, `decoy`, `approach`, `multi`.
