"""
Headless tests for the parts that must be correct independent of any UI or radio:
geometry, the safety bands, and the simulator's closed-loop behaviour.

Run:  python -m tests.test_core      (from the project root)
"""
from __future__ import annotations

import asyncio
import math

from backend.config import Config
from backend.formation import geometry as g
from backend.formation import safety as s
from backend.models import MidpointPose, VehicleProfile
from backend.sources.sim_source import SimLink
from backend.models import WorldTarget

PASS, FAIL = 0, 0


def check(name: str, cond: bool, detail: str = ""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ok   {name}")
    else:
        FAIL += 1
        print(f"  FAIL {name}  {detail}")


ORIGIN = (47.397742, 8.545594, 488.0)


def test_geo_roundtrip():
    print("geometry: ENU <-> lat/lon round-trip")
    for e, n, u in [(0, 0, 0), (50, -30, 10), (-12.5, 80, -5)]:
        lat, lon, alt = g.enu_to_latlon(e, n, u, ORIGIN)
        e2, n2, u2 = g.latlon_to_enu(lat, lon, alt, ORIGIN)
        err = math.dist((e, n, u), (e2, n2, u2))
        check(f"round-trip ({e},{n},{u}) err={err:.4f}m", err < 0.01, f"err={err}")


def test_formation_shapes():
    print("geometry: formation presets")
    line = g.build_formation("line", 2, separation_m=4.0, net=True, net_span_m=6.0)
    check("line/2 has 2 slots", len(line.slots) == 2)
    check("line/2 has 1 net", len(line.links) == 1)
    grid = g.build_formation("grid", 6, separation_m=3.0)
    check("grid/6 has 6 slots", len(grid.slots) == 6)
    vee = g.build_formation("vee", 5, separation_m=3.0)
    check("vee/5 has 5 slots", len(vee.slots) == 5)


def test_midpoint_geometry():
    print("geometry: midpoint -> slot positions")
    spec = g.build_formation("line", 2, separation_m=4.0)
    mid = MidpointPose(e=10.0, n=20.0, u=5.0, yaw_deg=0.0)
    pos = g.slot_world_positions(spec, mid)
    # heading 0 (north): line is lateral (east/west), drones 4m apart, centred on mid
    sep = math.dist(pos[0][:2], pos[1][:2])
    cx = (pos[0][0] + pos[1][0]) / 2
    cy = (pos[0][1] + pos[1][1]) / 2
    check(f"separation == 4m (got {sep:.3f})", abs(sep - 4.0) < 1e-6, f"{sep}")
    check("midpoint preserved", abs(cx - 10) < 1e-6 and abs(cy - 20) < 1e-6)
    check("both at midpoint altitude", abs(pos[0][2] - 5) < 1e-6)

    # rotate 90 deg: a lateral line should now lie north/south
    mid.yaw_deg = 90.0
    pos = g.slot_world_positions(spec, mid)
    dnorth = abs(pos[0][1] - pos[1][1])
    deast = abs(pos[0][0] - pos[1][0])
    check(f"after yaw90 line is N/S (dN={dnorth:.2f}, dE={deast:.2f})",
          dnorth > 3.9 and deast < 0.1, f"dN={dnorth} dE={deast}")


def test_targets_consistency():
    print("geometry: targets_for round-trips through GPS")
    spec = g.build_formation("line", 3, separation_m=5.0)
    mid = MidpointPose(e=0, n=0, u=8, yaw_deg=37.0)
    assign = {1: 0, 2: 1, 3: 2}
    targets = g.targets_for(spec, mid, ORIGIN, assign)
    expected = g.slot_world_positions(spec, mid)
    for sysid, slot in assign.items():
        e, n, u = g.latlon_to_enu(targets[sysid].lat, targets[sysid].lon,
                                  targets[sysid].alt_rel, ORIGIN)
        err = math.dist((e, n), expected[slot][:2])
        check(f"target sys{sysid} matches slot (err={err:.3f}m)", err < 0.02, f"{err}")


def test_safety_net_band():
    print("safety: net band + commanded-separation guard")
    cfg = Config()
    # separation 8m but net spans only 6m -> must block before flight
    spec = g.build_formation("line", 2, separation_m=8.0, net=True, net_span_m=6.0)
    viol = s.check_commanded_separation(spec)
    check("8m sep over 6m net is blocked", any(v.code == "sep_exceeds_net" for v in viol))
    # clamp it
    capped = s.clamp_separation(spec)
    check(f"clamp_separation caps to <=6m (got {capped:.2f})", capped <= 6.0 + 1e-6, f"{capped}")
    viol2 = s.check_commanded_separation(spec)
    check("after clamp, no violation", not viol2)


def test_safety_fence_clamp():
    print("safety: fence clamp keeps formation inside")
    cfg = Config()
    cfg.fence.radius_m = 50.0
    cfg.fence.alt_min_m, cfg.fence.alt_max_m = 2.0, 30.0
    spec = g.build_formation("line", 2, separation_m=4.0)
    mid = MidpointPose(e=200.0, n=0.0, u=100.0)  # way outside + too high
    s.clamp_midpoint(mid, spec, cfg)
    r = math.hypot(mid.e - cfg.fence.center_e, mid.n - cfg.fence.center_n)
    check(f"midpoint pulled inside fence (r={r:.1f} <= 50)", r <= 50.0 + 1e-6, f"{r}")
    check(f"altitude clamped to band (u={mid.u})", 2.0 <= mid.u <= 30.0, f"{mid.u}")


def test_sim_tracks_setpoint():
    print("sim: drone converges on a setpoint and holds")

    async def run():
        link = SimLink(2, ORIGIN, [VehicleProfile(), VehicleProfile()])
        await link.start()
        # target drone 1 to 20m east, 10m up
        tlat, tlon, _ = g.enu_to_latlon(20.0, 0.0, 10.0, ORIGIN)
        await link.arm(1, True)
        await link.set_mode(1, "GUIDED")
        await link.send_setpoint(1, WorldTarget(lat=tlat, lon=tlon, alt_rel=10.0, yaw_deg=90.0))
        for _ in range(200):  # ~10s sim time, enough to accelerate, cruise, settle
            await asyncio.sleep(0.05)
        snap = link.snapshot()
        e, n, _ = g.latlon_to_enu(snap[1].lat, snap[1].lon, 0.0, ORIGIN)
        u = snap[1].alt_rel
        await link.stop()
        return e, n, u, snap[1].heading, snap[1].battery_pct

    e, n, u, hdg, batt = asyncio.run(run())
    check(f"reached east≈20 (e={e:.1f})", abs(e - 20) < 1.5, f"e={e}")
    check(f"reached up≈10 (u={u:.1f})", abs(u - 10) < 1.0, f"u={u}")
    check(f"yawed toward 90 (hdg={hdg:.0f})", abs(((hdg - 90 + 180) % 360) - 180) < 10, f"{hdg}")
    check(f"battery drained from arming (batt={batt:.1f})", batt < 100.0, f"{batt}")


def test_prop_geometry():
    print("models: prop / net geometry")
    normal = VehicleProfile()                       # props up, net underneath
    check("normal quad props on top", normal.props_side == "top")
    check("normal quad net clear (net below props)", normal.net_clear_of_props)
    inv = VehicleProfile(inverted=True, net_attach="top")   # props down, top free
    check("inverted props on bottom", inv.props_side == "bottom")
    check("inverted top-free net clear", inv.net_clear_of_props)
    exposed = VehicleProfile(net_attach="top")       # props up AND net up
    check("net on prop side is exposed", not exposed.net_clear_of_props)


def _state_at(sysid, e, n, u, prof_id="default"):
    from backend.models import VehicleState, Autopilot
    import time as _t
    lat, lon, alt = g.enu_to_latlon(e, n, u, ORIGIN)
    return VehicleState(sysid=sysid, name=f"D{sysid}", autopilot=Autopilot.SIM,
                        profile_id=prof_id, lat=lat, lon=lon, alt_msl=alt, alt_rel=u,
                        fix_type=3, last_heartbeat=_t.monotonic())


def test_safety_props_clear():
    print("safety: net slack only matters when props are exposed")
    cfg = Config()
    small = VehicleProfile("small", span_m=0.2)      # radius .1 -> collision_min 1.2
    cfg.profiles["small"] = small
    spec = g.build_formation("line", 2, separation_m=2.0, net=True,
                             net_span_m=6.0, net_slack_min_m=2.0)
    assign = {1: 0, 2: 1}

    # 1.5 m apart: inside slack band (2.0) but clear of collision (1.2)
    states = [_state_at(1, 0, 0, 5, "small"), _state_at(2, 1.5, 0, 5, "small")]
    profs = {1: small, 2: small}            # net_attach default "bottom", props top -> CLEAR
    v_clear = s.check_separation(states, profs, spec, assign, ORIGIN)
    check("props clear -> no net_slack warning", not any(x.code == "net_slack" for x in v_clear))
    check("props clear -> no block at 1.5m", not any(x.severity == "block" for x in v_clear))

    exposed = VehicleProfile("exp", span_m=0.2, net_attach="top")   # props top, net top
    profs_e = {1: exposed, 2: exposed}
    v_exp = s.check_separation(states, profs_e, spec, assign, ORIGIN)
    check("props exposed -> net_slack warning", any(x.code == "net_slack" for x in v_exp))

    # 1.0 m apart: below collision_min -> hard block regardless of prop side
    near = [_state_at(1, 0, 0, 5, "small"), _state_at(2, 1.0, 0, 5, "small")]
    v_near = s.check_separation(near, profs, spec, assign, ORIGIN)
    check("collision still blocks even on a net pair",
          any(x.code == "too_close" and x.severity == "block" for x in v_near))


def test_enu_altitude():
    print("geometry: vehicle ENU up = relative altitude (regression guard)")
    from backend.models import VehicleState, Autopilot
    import time as _t
    origin = (47.0, 8.0, 488.0)               # realistic high MSL origin
    lat, lon, _ = g.enu_to_latlon(30.0, 0.0, 10.0, origin)
    v = VehicleState(sysid=1, name="A", autopilot=Autopilot.SIM, profile_id="default",
                     lat=lat, lon=lon, alt_msl=498.0, alt_rel=10.0, fix_type=3,
                     last_heartbeat=_t.monotonic())
    e, n, u = g.enu_of(v, origin)
    check("up equals relative altitude (not alt - origin MSL)", abs(u - 10.0) < 1e-6, f"(u={u:.1f})")
    check("east recovered from lat/lon", abs(e - 30.0) < 0.5, f"(e={e:.2f})")


if __name__ == "__main__":
    for fn in [test_geo_roundtrip, test_formation_shapes, test_midpoint_geometry,
               test_targets_consistency, test_enu_altitude, test_safety_net_band,
               test_safety_fence_clamp, test_prop_geometry, test_safety_props_clear,
               test_sim_tracks_setpoint]:
        fn()
    print(f"\n{PASS} passed, {FAIL} failed")
    raise SystemExit(1 if FAIL else 0)
