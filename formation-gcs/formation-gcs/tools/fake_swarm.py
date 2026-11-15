"""
Live MAVLink demo — watch the real link drive the dashboard.

Starts N fake ArduCopter endpoints that speak genuine MAVLink over UDP, runs the station
against them as if they were real drones, and opens the dashboard. Use it to *see* the
MAVLink path (not the sim) work: open the browser, hit Auto take-off, and watch the staged
sequence in the 2D plan and 3D view as the station discovers, arms, modes and commands the
fleet over MAVLink.

    python -m tools.fake_swarm --count 3
    # browser opens; press Auto take-off

This is a stub fleet (faithful telemetry, crude dynamics). For real flight dynamics, point
run.py at ArduPilot SITL instead.
"""
from __future__ import annotations

import argparse
import threading
import webbrowser

from aiohttp import web

from backend.app import make_app
from backend.config import Config
from tools.fake_autopilot import FakeCopter

# spread the fleet a few metres apart near the ArduPilot SITL default origin (Zurich)
BASE_LAT, BASE_LON = 47.397000, 8.545000


def main():
    ap = argparse.ArgumentParser(description="Fake MAVLink swarm + dashboard")
    ap.add_argument("--count", type=int, default=3, help="number of fake drones (2-8)")
    ap.add_argument("--port", type=int, default=8765, help="dashboard port")
    ap.add_argument("--base-udp", type=int, default=14570, help="first UDP port")
    ap.add_argument("--inverted", action="store_true",
                    help="present the fleet as props-down / top-free rigs")
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    n = max(2, min(8, args.count))
    fakes, conns, names = [], [], {}
    for i in range(n):
        sid = i + 1
        port = args.base_udp + sid
        # ~3 m apart in longitude so they start as a tight ground line
        lon = BASE_LON + i * 0.00004
        fakes.append(FakeCopter(sid, f"udpout:127.0.0.1:{port}", BASE_LAT, lon))
        conns.append(f"udpin:0.0.0.0:{port}")
        names[sid] = ["Alpha", "Bravo", "Charlie", "Delta", "Echo", "Foxtrot",
                      "Golf", "Hotel"][i]
    for f in fakes:
        f.start()

    cfg = Config()
    cfg.use_sim = False
    cfg.connections = conns
    cfg.names = names
    if args.inverted:
        for p in cfg.profiles.values():
            p.inverted = True
            p.net_attach = "top"

    app = make_app(cfg)
    url = f"http://127.0.0.1:{args.port}"
    print(f"\n  Fake MAVLink swarm up: {n} drones over UDP {args.base_udp + 1}..{args.base_udp + n}")
    print(f"  Dashboard: {url}")
    print("  In the browser: press Auto take-off and watch the 2D / 3D views.\n")
    if not args.no_browser:
        threading.Timer(1.2, lambda: webbrowser.open(url)).start()
    web.run_app(app, host="127.0.0.1", port=args.port, print=None)


if __name__ == "__main__":
    main()
