"""
Independent position cross-check.

The point of this test is *bias removal*. The station computes each drone's local ENU
position from its GPS with its own geometry code. Here we take the positions the station
reports over its websocket and re-derive them with an entirely separate, authoritative
library — pyproj (the PROJ geodesy engine that QGIS, GDAL and friends use). If the two
agree, the station's decode and projection are correct independently of its own maths.

What this checks, per vehicle:
  1. DECODE   — the lat/lon the station reports equals what the vehicle actually emitted.
  2. GEOMETRY — the station's local E/N (its own equirectangular projection) matches
                pyproj's geodesic E/N from the same lat/lon and origin, and U == reported
                relative altitude.

The drones are held static at known, distinct positions so the comparison is exact.

Run:  python -m tests.position_crosscheck
"""
from __future__ import annotations

import asyncio
import math

import aiohttp
from aiohttp import web
from pyproj import Geod

from backend.app import make_app
from backend.config import Config
from tools.fake_autopilot import FakeCopter

GEOD = Geod(ellps="WGS84")
PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1; print(f"  ok   {name}  {detail}")
    else:
        FAIL += 1; print(f"  FAIL {name}  {detail}")


def enu_pyproj(lat, lon, alt_rel, origin):
    """Independent local ENU: geodesic azimuth+distance from origin (PROJ, WGS84)."""
    olat, olon, _ = origin
    az12, _, dist = GEOD.inv(olon, olat, lon, lat)   # az clockwise from north, metres
    return (dist * math.sin(math.radians(az12)),
            dist * math.cos(math.radians(az12)),
            alt_rel)


async def serve(cfg, port):
    app = make_app(cfg); runner = web.AppRunner(app); await runner.setup()
    await web.TCPSite(runner, "127.0.0.1", port).start()
    return runner


async def main():
    print("position cross-check: station ENU vs pyproj (PROJ) on a static fleet")
    # three drones held at distinct, known positions and altitudes (no motion)
    truth = {
        1: (47.397000, 8.545000, 5.0),
        2: (47.397420, 8.545600, 8.0),
        3: (47.396780, 8.544700, 12.0),
    }
    fakes = [FakeCopter(sid, f"udpout:127.0.0.1:{14580 + sid}", lat, lon, alt_rel=alt)
             for sid, (lat, lon, alt) in truth.items()]
    for f in fakes:
        f.start()

    cfg = Config()
    cfg.use_sim = False
    cfg.connections = [f"udpin:0.0.0.0:{14580 + sid}" for sid in truth]
    cfg.names = {1: "Alpha", 2: "Bravo", 3: "Charlie"}

    runner = await serve(cfg, 8807)
    latest = {}
    try:
        async with aiohttp.ClientSession() as s:
            async with s.ws_connect("http://127.0.0.1:8807/ws") as ws:
                end = asyncio.get_event_loop().time() + 4
                while asyncio.get_event_loop().time() < end:
                    try:
                        m = await asyncio.wait_for(ws.receive(), 4)
                    except asyncio.TimeoutError:
                        break
                    if m.type == aiohttp.WSMsgType.TEXT:
                        d = m.json()
                        if d["type"] == "state" and d.get("origin"):
                            latest = d

        check("station produced a fix and origin", bool(latest), "")
        if not latest:
            raise SystemExit(1)

        o = latest["origin"]
        origin = (o["lat"], o["lon"], o["alt"])
        print(f"  origin (first fix): {origin[0]:.6f}, {origin[1]:.6f}")
        print(f"  {'sys':>3} {'station lat/lon':>22}   {'station E,N,U':>20}   "
              f"{'pyproj E,N':>16}   {'Δhoriz':>7}")

        worst_decode = worst_geom = 0.0
        for v in sorted(latest["vehicles"], key=lambda x: x["sysid"]):
            sid = v["sysid"]
            tlat, tlon, talt = truth[sid]
            # 1) decode: reported lat/lon vs emitted truth
            dlat = abs(v["lat"] - tlat); dlon = abs(v["lon"] - tlon)
            worst_decode = max(worst_decode, dlat, dlon)
            # 2) geometry: station E/N vs pyproj E/N from the station's own lat/lon
            pe, pn, pu = enu_pyproj(v["lat"], v["lon"], v["alt"], origin)
            dh = math.hypot(v["e"] - pe, v["n"] - pn)
            worst_geom = max(worst_geom, dh, abs(v["u"] - pu))
            print(f"  {sid:>3} {v['lat']:>11.6f},{v['lon']:>10.6f}   "
                  f"{v['e']:>6.2f},{v['n']:>6.2f},{v['u']:>5.2f}   "
                  f"{pe:>7.2f},{pn:>7.2f}   {dh:>6.3f}m")

        check("station lat/lon matches emitted truth", worst_decode < 1e-5,
              f"(max Δ {worst_decode:.2e}°)")
        check("station ENU matches pyproj geodesy", worst_geom < 0.5,
              f"(max Δ {worst_geom:.3f} m over ~50 m baseline)")
    finally:
        for f in fakes:
            f.stop()
        await runner.cleanup()

    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
