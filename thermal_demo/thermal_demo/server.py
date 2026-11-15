"""
server.py — web UI for the thermal detection demo.

Runs the detect -> track -> guide pipeline on synthetic thermal frames in a
background thread, and serves:
  /              the dashboard (HTML)
  /stream.mjpg   live annotated video (MJPEG, works in any browser)
  /stats.json    live numbers (latency, lock state, counts)

Standard library only (http.server) so it runs on a Raspberry Pi Zero 2W with
no web framework. Connect from any browser on the same network:
    http://<pi-ip>:8000      (or http://localhost:8000 on a laptop)

Run:
    python3 server.py                 # default, port 8000, loops forever
    python3 server.py --port 8080
    python3 server.py --fps 25
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from frame_source import SyntheticThermalSource
from scenarios import make_scenario, SCENARIOS
from detector import ThermalDetector
from tracker import KalmanTracker, PNGuidance
import render

clock = time.perf_counter_ns
NS = 1e6


class Pipeline:
    """Runs the detection chain in a loop; holds the latest annotated JPEG + stats."""

    def __init__(self, fps=25, loop=True, scenario="decoy"):
        self.fps = fps
        self.loop = loop
        self.scenario = scenario
        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.stats = {"frame": 0, "state": "starting", "algo_ms": 0.0,
                      "e2e_ms": 0.0, "cmd_dps": 0.0, "lock": 0, "total": 0,
                      "lock_pct": 0.0, "mean_algo_ms": 0.0, "err_px": 0.0,
                      "capture_real": False}
        self._algo_hist = []
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        while self._running:
            src = make_scenario(self.scenario, n_frames=300, seed=42)
            det = ThermalDetector()
            kf = KalmanTracker()
            pn = PNGuidance()
            lock_count = 0
            total = 0
            errs = []
            period = 1.0 / self.fps

            while self._running:
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

                algo_ms = (t4 - t1) / NS
                e2e_ms = (t4 - t0) / NS
                self._algo_hist.append(algo_ms)
                if len(self._algo_hist) > 600:
                    self._algo_hist.pop(0)

                locked = (meas is not None and state is not None)
                total += 1
                if locked:
                    lock_count += 1
                    if truth is not None:
                        e = ((state[0] - truth[0]) ** 2 + (state[1] - truth[1]) ** 2) ** 0.5
                        errs.append(e)

                img = render.annotate(frame, meas, state, locked, src.i,
                                      algo_ms, e2e_ms, cmd, lock_count, total)
                jpeg = render.to_jpeg(img)

                mean_algo = sum(self._algo_hist) / len(self._algo_hist)
                with self.lock:
                    if jpeg is not None:
                        self.latest_jpeg = jpeg
                    self.stats = {
                        "frame": src.i,
                        "state": "LOCKED" if locked else "SEARCHING",
                        "algo_ms": round(algo_ms, 2),
                        "e2e_ms": round(e2e_ms, 2),
                        "cmd_dps": round(cmd, 1),
                        "lock": lock_count,
                        "total": total,
                        "lock_pct": round(100 * lock_count / max(total, 1), 1),
                        "mean_algo_ms": round(mean_algo, 2),
                        "err_px": round(sum(errs) / len(errs), 2) if errs else 0.0,
                        "capture_real": src.capture_is_real,
                    }
                # pace to target fps
                dt = (clock() - t0) / 1e9
                if dt < period:
                    time.sleep(period - dt)

            if not self.loop:
                self._running = False


PIPELINE = None  # set in main


PAGE = """<!doctype html><html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Thermal Interceptor — Live Detection</title>
<style>
  :root{ --bg:#0d1117; --panel:#161b22; --line:#30363d; --txt:#e6edf3;
         --accent:#ff7b39; --ok:#3fb950; --dim:#8b949e; }
  *{box-sizing:border-box} body{margin:0;background:var(--bg);color:var(--txt);
    font-family:'DejaVu Sans Mono',ui-monospace,Menlo,Consolas,monospace}
  header{padding:14px 20px;border-bottom:1px solid var(--line);display:flex;
    align-items:baseline;gap:14px}
  header h1{font-size:16px;margin:0;letter-spacing:.5px}
  header .sub{color:var(--dim);font-size:12px}
  .wrap{display:flex;gap:18px;padding:18px;flex-wrap:wrap}
  .video{flex:2 1 520px;min-width:320px}
  .video img{width:100%;border:1px solid var(--line);border-radius:8px;display:block;
    background:#000}
  .side{flex:1 1 240px;min-width:220px;display:flex;flex-direction:column;gap:12px}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px}
  .card h2{margin:0 0 10px;font-size:12px;color:var(--dim);text-transform:uppercase;
    letter-spacing:1px;font-weight:600}
  .stat{display:flex;justify-content:space-between;padding:5px 0;font-size:14px}
  .stat .v{font-weight:700}
  .big{font-size:30px;font-weight:800;letter-spacing:1px}
  .pill{display:inline-block;padding:3px 10px;border-radius:20px;font-size:12px;font-weight:700}
  .pill.lock{background:rgba(63,185,80,.15);color:var(--ok);border:1px solid var(--ok)}
  .pill.search{background:rgba(255,123,57,.12);color:var(--accent);border:1px solid var(--accent)}
  .note{font-size:11px;color:var(--dim);line-height:1.5}
  .accent{color:var(--accent)} .ok{color:var(--ok)}
</style></head><body>
<header>
  <h1>THERMAL INTERCEPTOR <span class="accent">// LIVE</span></h1>
  <span class="sub">synthetic thermal feed &middot; classical CV + Kalman + PN guidance</span>
</header>
<div class="wrap">
  <div class="video">
    <img src="/stream.mjpg" alt="live detection stream">
  </div>
  <div class="side">
    <div class="card">
      <h2>Lock state</h2>
      <div id="state" class="big">—</div>
      <div style="margin-top:8px"><span id="pill" class="pill search">…</span></div>
    </div>
    <div class="card">
      <h2>Latency (measured)</h2>
      <div class="stat"><span>algorithm</span><span class="v accent"><span id="algo">—</span> ms</span></div>
      <div class="stat"><span>mean (algorithm)</span><span class="v"><span id="malgo">—</span> ms</span></div>
      <div class="stat"><span>end-to-end*</span><span class="v"><span id="e2e">—</span> ms</span></div>
    </div>
    <div class="card">
      <h2>Tracking</h2>
      <div class="stat"><span>frame</span><span class="v" id="frame">—</span></div>
      <div class="stat"><span>locked</span><span class="v"><span id="lock">—</span></span></div>
      <div class="stat"><span>lock rate</span><span class="v ok"><span id="lockpct">—</span>%</span></div>
      <div class="stat"><span>track error</span><span class="v"><span id="err">—</span> px</span></div>
      <div class="stat"><span>PN command</span><span class="v"><span id="cmd">—</span> &deg;/s</span></div>
    </div>
    <div class="card">
      <div class="note">* end-to-end includes a <b>synthetic</b> capture stage on this
      machine, not the real camera. The <span class="accent">algorithm</span> figure is
      what transfers to the Raspberry Pi; the real camera-capture delay is a hardware
      measurement (Recommendation&nbsp;R1).</div>
    </div>
  </div>
</div>
<script>
async function poll(){
  try{
    const r = await fetch('/stats.json',{cache:'no-store'});
    const s = await r.json();
    document.getElementById('state').textContent = s.state;
    const pill = document.getElementById('pill');
    if(s.state==='LOCKED'){pill.className='pill lock';pill.textContent='● TARGET LOCK';}
    else{pill.className='pill search';pill.textContent='○ SEARCHING';}
    document.getElementById('algo').textContent = s.algo_ms;
    document.getElementById('malgo').textContent = s.mean_algo_ms;
    document.getElementById('e2e').textContent = s.e2e_ms;
    document.getElementById('frame').textContent = s.frame;
    document.getElementById('lock').textContent = s.lock+'/'+s.total;
    document.getElementById('lockpct').textContent = s.lock_pct;
    document.getElementById('err').textContent = s.err_px;
    document.getElementById('cmd').textContent = s.cmd_dps;
  }catch(e){}
}
setInterval(poll, 250); poll();
</script>
</body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass  # quiet

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path.startswith("/stats.json"):
            with PIPELINE.lock:
                body = json.dumps(PIPELINE.stats).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path.startswith("/stream.mjpg"):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with PIPELINE.lock:
                        jpeg = PIPELINE.latest_jpeg
                    if jpeg is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                    time.sleep(1.0 / 25)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(404)
            self.end_headers()


def main():
    global PIPELINE
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--scenario", default="decoy", choices=list(SCENARIOS),
                    help="which synthetic clip to run (default: decoy)")
    args = ap.parse_args()

    if not render.HAVE_CV2:
        print("WARNING: OpenCV not found — install with:")
        print("   pip install opencv-python-headless")
        print("The video stream needs it for drawing + JPEG encoding.\n")

    PIPELINE = Pipeline(fps=args.fps, loop=True, scenario=args.scenario)
    t = threading.Thread(target=PIPELINE.run, daemon=True)
    t.start()

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print("=" * 56)
    print(" Thermal detection web UI is running.")
    print(f"   scenario        : {args.scenario}")
    print(f"   On this machine : http://localhost:{args.port}")
    print(f"   On the network  : http://<this-device-ip>:{args.port}")
    print(" Press Ctrl+C to stop.")
    print("=" * 56)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping…")
        PIPELINE.stop()
        srv.shutdown()


if __name__ == "__main__":
    main()
