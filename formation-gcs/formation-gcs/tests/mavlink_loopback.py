"""
Real-MAVLink loopback validation.

This exercises the ACTUAL MAVLinkLink code path over a real MAVLink-over-UDP wire,
with no autopilot or SITL involved. Each "drone" is a tiny pymavlink endpoint that
behaves like an ArduCopter: it streams HEARTBEAT / GLOBAL_POSITION_INT / GPS_RAW_INT /
SYS_STATUS, answers a capabilities request with AUTOPILOT_VERSION, and reacts to the
messages our station sends — SET_MODE, MAV_CMD_COMPONENT_ARM_DISARM, and
SET_POSITION_TARGET_GLOBAL_INT — moving toward each commanded setpoint.

If this passes, the software side of the real link works end to end: discovery, frame/
autopilot identification, telemetry decode, mode/arm commands, guided setpoints and the
closed control loop. What remains for real flight is purely vehicle-side configuration.

Run:  python -m tests.mavlink_loopback
"""
from __future__ import annotations

import asyncio
import math
import threading
import time

import aiohttp
from aiohttp import web

from backend.app import make_app
from backend.config import Config
from tools.fake_autopilot import FakeCopter

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1; print(f"  ok   {name}  {detail}")
    else:
        FAIL += 1; print(f"  FAIL {name}  {detail}")


async def serve(cfg, port):
    app = make_app(cfg); runner = web.AppRunner(app); await runner.setup()
    await web.TCPSite(runner, "127.0.0.1", port).start()
    return runner


async def drain(ws, seconds, on_msg):
    end = asyncio.get_event_loop().time() + seconds
    while asyncio.get_event_loop().time() < end:
        try:
            msg = await asyncio.wait_for(ws.receive(), timeout=seconds)
        except asyncio.TimeoutError:
            return
        if msg.type == aiohttp.WSMsgType.TEXT:
            on_msg(msg.json())


async def main():
    print("real-MAVLink loopback: station ↔ two fake ArduCopters over UDP")
    fakes = [FakeCopter(1, "udpout:127.0.0.1:14571", 47.39700, 8.54500),
             FakeCopter(2, "udpout:127.0.0.1:14572", 47.39700, 8.54504)]
    for f in fakes:
        f.start()

    cfg = Config()
    cfg.use_sim = False
    cfg.connections = ["udpin:0.0.0.0:14571", "udpin:0.0.0.0:14572"]
    cfg.names = {1: "Alpha", 2: "Bravo"}
    cfg.sysid_profiles = {1: "8inch", 2: "10inch"}
    cfg.sequence.hover_alt_m = 2.0
    cfg.sequence.work_alt_m = 4.0
    cfg.sequence.ascent_rate_ms = 2.0
    cfg.sequence.climb_rate_ms = 3.0
    cfg.sequence.descent_rate_ms = 3.0
    cfg.sequence.settle_s = 0.4

    runner = await serve(cfg, 8806)
    roster_seen, phases = {}, set()
    last_state = {}
    try:
        async with aiohttp.ClientSession() as s:
            async with s.ws_connect("http://127.0.0.1:8806/ws") as ws:
                def rec(d):
                    if d["type"] == "roster":
                        for v in d["vehicles"]:
                            roster_seen[v["sysid"]] = v
                    elif d["type"] == "state":
                        phases.add(d["phase"]); last_state.clear(); last_state.update(d)
                # 1) discovery + telemetry over real MAVLink
                await drain(ws, 4, rec)
                check("both vehicles discovered", set(roster_seen) == {1, 2}, f"(saw {sorted(roster_seen)})")
                if 1 in roster_seen:
                    v = roster_seen[1]
                    check("identified as ArduPilot", v["autopilot"] == "ardupilot", f"({v['autopilot']})")
                    check("frame decoded as quad", v["frame"] == "quad", f"({v['frame']})")
                    check("GPS fix decoded", v["fix"] is True)
                    check("online via heartbeat", v["online"] is True)
                    check("profile matched from config", v["profile"]["id"] == "8inch")

                # 2) staged auto take-off — exercises SET_MODE + ARM + setpoints
                await ws.send_json({"type": "command", "name": "auto_takeoff", "alt": 4.0})
                await drain(ws, 16, rec)
                check("ARM command reached the drone", all(f.got_arm_cmd for f in fakes))
                check("drones armed from our command", all(f.armed for f in fakes))
                check("GUIDED mode set on the drone", all(f.custom_mode == 4 for f in fakes),
                      f"(modes {[f.custom_mode for f in fakes]})")
                check("guided setpoints received", all(f.setpoints > 5 for f in fakes),
                      f"(counts {[f.setpoints for f in fakes]})")
                check("sequence reached AIRBORNE", "airborne" in phases, f"(saw {sorted(phases)})")
                check("GUIDED takeoff was commanded (not position-only)",
                      all(f.got_takeoff for f in fakes))
                check("no auto-disarm on the ground", not any(f.auto_disarmed for f in fakes))
                climbed = [f.alt_rel for f in fakes]
                check("drones climbed toward working alt", all(a > 3.0 for a in climbed),
                      f"(alts {[round(a,1) for a in climbed]})")

                # 3) staged auto land — back down and disarm
                await ws.send_json({"type": "command", "name": "auto_land"})
                landed = {"disarmed": False}
                def rec2(d):
                    if d["type"] == "state" and d["phase"] == "disarmed":
                        landed["disarmed"] = True
                await drain(ws, 16, rec2)
                check("sequence returned to DISARMED", landed["disarmed"])
                check("drones disarmed by land sequence", all(not f.armed for f in fakes))
    finally:
        for f in fakes:
            f.stop()
        await runner.cleanup()

    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
