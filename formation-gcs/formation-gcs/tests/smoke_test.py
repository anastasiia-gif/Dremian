"""
End-to-end smoke tests of the live server in sim mode.

Covers the three things that can only be verified with the real loop running:
  1. the staged take-off / land sequencer walks through its phases and lands disarmed,
  2. the operator can fly the midpoint and set spacing once airborne,
  3. the server can be reconfigured (fleet size) at runtime.

Run:  python -m tests.smoke_test
"""
from __future__ import annotations

import asyncio
import math

import aiohttp
from aiohttp import web

from backend.app import make_app
from backend.config import load_config

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1; print(f"  ok   {name}  {detail}")
    else:
        FAIL += 1; print(f"  FAIL {name}  {detail}")


async def serve(cfg, port):
    app = make_app(cfg)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", port)
    await site.start()
    return runner


async def drain(ws, seconds, on_msg):
    """Read messages for `seconds`, calling on_msg(data) for each state/roster/event."""
    end = asyncio.get_event_loop().time() + seconds
    while asyncio.get_event_loop().time() < end:
        try:
            msg = await asyncio.wait_for(ws.receive(), timeout=seconds)
        except asyncio.TimeoutError:
            return
        if msg.type == aiohttp.WSMsgType.TEXT:
            on_msg(msg.json())


# --------------------------------------------------------------------------- #
async def test_sequencer():
    print("sequencer: staged take-off → fly → staged land")
    cfg = load_config(None)
    cfg.use_sim = True; cfg.sim_count = 3
    cfg.names = {1: "Alpha", 2: "Bravo", 3: "Charlie"}
    # speed the sequence up so the test runs in a few seconds
    cfg.sequence.hover_alt_m = 2.0
    cfg.sequence.work_alt_m = 4.0
    cfg.sequence.ascent_rate_ms = 2.0
    cfg.sequence.climb_rate_ms = 3.0
    cfg.sequence.descent_rate_ms = 2.5
    cfg.sequence.settle_s = 0.4

    runner = await serve(cfg, 8801)
    phases = set()
    last = {}
    try:
        async with aiohttp.ClientSession() as s:
            async with s.ws_connect("http://127.0.0.1:8801/ws") as ws:
                def rec(d):
                    if d["type"] == "state":
                        phases.add(d["phase"]); last.clear(); last.update(d)
                await ws.send_json({"type": "command", "name": "auto_takeoff", "alt": 4.0})
                await drain(ws, 14, rec)            # let it reach airborne
                check("reached AIRBORNE", "airborne" in phases, f"(saw {sorted(phases)})")
                check("passed through ascending", "ascending" in phases)
                check("passed through spreading", "spreading" in phases)
                check("passed through climbing", "climbing" in phases)
                if last:
                    check(f"midpoint climbed to ~4 m (u={last['midpoint']['u']})",
                          abs(last["midpoint"]["u"] - 4.0) < 0.8)
                    alts = [v["alt"] for v in last["vehicles"] if v["alt"] is not None]
                    check(f"drones near working alt (min={min(alts):.1f})", min(alts) > 3.0)

                # now land
                await ws.send_json({"type": "command", "name": "auto_land"})
                landed = {"done": False, "armed_after": True}
                def rec2(d):
                    if d["type"] == "state":
                        if d["phase"] == "disarmed":
                            landed["done"] = True
                            landed["armed_after"] = any(v["armed"] for v in d["vehicles"])
                await drain(ws, 14, rec2)
                check("returned to DISARMED", landed["done"])
                check("all drones disarmed after landing", not landed["armed_after"])
    finally:
        await runner.cleanup()


async def test_fly_and_spacing():
    print("airborne: fly the midpoint and set spacing")
    cfg = load_config(None)
    cfg.use_sim = True; cfg.sim_count = 3
    runner = await serve(cfg, 8802)
    mid_track, sep_reached = [], [None]
    try:
        async with aiohttp.ClientSession() as s:
            async with s.ws_connect("http://127.0.0.1:8802/ws") as ws:
                await ws.send_json({"type": "command", "name": "engage"})   # manual, fast
                await ws.send_json({"type": "command", "name": "arm"})       # so they can fly
                await ws.send_json({"type": "set_formation", "kind": "line", "n": 3,
                                    "separation": 5.0, "net": True, "net_span": 6.0})
                await ws.send_json({"type": "input", "fwd": 2.0})

                def rec(d):
                    if d["type"] == "state":
                        mid_track.append((d["midpoint"]["e"], d["midpoint"]["n"]))
                        vs = sorted([v for v in d["vehicles"] if v["e"] is not None],
                                    key=lambda v: v["slot"] if v["slot"] is not None else 9)
                        if len(vs) >= 2:
                            seps = [math.dist((vs[i]["e"], vs[i]["n"]), (vs[i+1]["e"], vs[i+1]["n"]))
                                    for i in range(len(vs)-1)]
                            m = min(seps)
                            sep_reached[0] = m if sep_reached[0] is None else max(sep_reached[0], m)
                await drain(ws, 9, rec)
                await ws.send_json({"type": "input", "fwd": 0.0})
        check("adjacent spacing reached ~5 m",
              sep_reached[0] is not None and abs(sep_reached[0] - 5.0) < 1.2,
              f"(={sep_reached[0]:.2f})" if sep_reached[0] else "")
        moved = (math.dist(mid_track[0], mid_track[-1]) if len(mid_track) >= 2 else 0.0)
        check("midpoint translated when flown forward", moved > 2.0, f"(moved {moved:.1f} m)")
    finally:
        await runner.cleanup()


async def test_reconfigure():
    print("reconfigure: change fleet size at runtime")
    cfg = load_config(None)
    cfg.use_sim = True; cfg.sim_count = 3
    runner = await serve(cfg, 8803)
    counts = []
    try:
        async with aiohttp.ClientSession() as s:
            async with s.ws_connect("http://127.0.0.1:8803/ws") as ws:
                def rec(d):
                    if d["type"] == "roster":
                        counts.append(len(d["vehicles"]))
                await drain(ws, 2, rec)
                first = max(counts) if counts else 0
                await ws.send_json({"type": "command", "name": "configure",
                                    "config": {"mode": "sim", "sim_count": 5,
                                               "names": {"4": "Delta", "5": "Echo"}}})
                counts.clear()
                await drain(ws, 3, rec)
                second = max(counts) if counts else 0
        check(f"started with 3 ({first})", first == 3)
        check(f"reconfigured to 5 ({second})", second == 5)
    finally:
        await runner.cleanup()


async def main():
    await test_sequencer()
    await test_fly_and_spacing()
    await test_reconfigure()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
