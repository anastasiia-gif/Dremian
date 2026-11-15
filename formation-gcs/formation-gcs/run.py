#!/usr/bin/env python3
"""
formation-gcs entry point.

Examples
--------
    python run.py --sim --count 3            # 3 simulated drones, no hardware
    python run.py --connect udp:127.0.0.1:14550 --connect udp:127.0.0.1:14560
    python run.py                            # use whatever config.json says

Then open http://127.0.0.1:8765
"""
from __future__ import annotations

import argparse
import logging

from aiohttp import web

from backend.app import make_app
from backend.config import load_config


def main() -> None:
    p = argparse.ArgumentParser(description="Multi-drone formation ground station")
    p.add_argument("--config", default="config.json", help="path to config.json")
    p.add_argument("--sim", action="store_true", help="force the simulator")
    p.add_argument("--count", type=int, help="number of simulated drones")
    p.add_argument("--connect", action="append", metavar="CONNSTR",
                   help="real connection string (repeatable), e.g. udp:127.0.0.1:14550")
    p.add_argument("--host", help="bind host")
    p.add_argument("--port", type=int, help="bind port")
    p.add_argument("--no-browser", action="store_true", help="don't auto-open the dashboard")
    args = p.parse_args()

    cfg = load_config(args.config)
    if args.sim:
        cfg.use_sim = True
    if args.count:
        cfg.sim_count = args.count
    if args.connect:
        cfg.connections = args.connect
        cfg.use_sim = False
    if args.host:
        cfg.ws_host = args.host
    if args.port:
        cfg.ws_port = args.port

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")

    mode = f"SIM ×{cfg.sim_count}" if cfg.use_sim else f"{len(cfg.connections)} link(s)"
    host = "127.0.0.1" if cfg.ws_host in ("0.0.0.0", "::") else cfg.ws_host
    url = f"http://{host}:{cfg.ws_port}"
    log = logging.getLogger("formation")
    log.info("starting — %s — open %s", mode, url)

    if not args.no_browser:
        import threading
        import webbrowser
        threading.Timer(1.2, lambda: webbrowser.open(url)).start()

    web.run_app(make_app(cfg), host=cfg.ws_host, port=cfg.ws_port, print=None)


if __name__ == "__main__":
    main()
