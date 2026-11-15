"""
The application: control loop + websocket server + flight-phase sequencer.

The loop pulls a snapshot from the link, updates discovery, fixes the origin, runs the
flight phase machine, evaluates safety, and pushes guided setpoints. It broadcasts the
whole picture to every dashboard.

Flight phases (the staged take-off / landing that keeps a netted group from yanking the
cloth into its own props):

    disarmed → arming → ascending → spreading → climbing → AIRBORNE
                                                              │
                                          airborne → descending → landing → settling → disarmed

  ascending   all drones rise together, holding their ground positions, to a low hover
              so the net lifts evenly (no single drone dragging it).
  spreading   at hover, open from tight ground spacing to the commanded separation.
  climbing    rise together to the working altitude, then hand control to the operator.
  airborne    operator flies the midpoint (forward/right/up/yaw); separation live.
  descending  lower the whole formation back to hover.
  landing     descend straight down, holding horizontal, to the ground.
  settling    idle briefly, then disarm.

The operator flies only in AIRBORNE; every other phase is driven automatically.
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import time

from aiohttp import WSMsgType, web

from .config import Config, FenceConfig
from .discovery import Discovery, RosterEntry
from .formation import geometry, safety
from .models import FormationSpec, MidpointPose, VehicleProfile, WorldTarget
from .protocol import (event_message, parse_command, roster_message, state_message)
from .sources.base import VehicleLink

log = logging.getLogger("formation")
FRONTEND = os.path.join(os.path.dirname(os.path.dirname(__file__)), "frontend")
SIM_ORIGIN = (47.397742, 8.545594, 488.0)   # ArduPilot SITL default (Zurich)

ACTIVE_PHASES = {"arming", "ascending", "spreading", "climbing", "airborne",
                 "descending", "landing"}
FORMATION_PHASES = {"spreading", "climbing", "airborne", "descending"}


def build_link(cfg: Config) -> VehicleLink:
    if cfg.use_sim:
        from .sources.sim_source import SimLink
        origin = (cfg.origin_lat or SIM_ORIGIN[0], cfg.origin_lon or SIM_ORIGIN[1],
                  cfg.origin_alt or SIM_ORIGIN[2])
        profiles = [cfg.profile_for(i + 1) for i in range(cfg.sim_count)]
        return SimLink(cfg.sim_count, origin, profiles, cfg.names)
    from .sources.mavlink_source import MAVLinkLink
    return MAVLinkLink(cfg.connections, cfg.names, cfg.sysid_profiles)


class Controller:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.link = build_link(cfg)
        self.discovery = Discovery(cfg, on_join=self._on_join, on_leave=self._on_leave)

        self._form = {"kind": "line", "net": True, "net_span": 6.0}
        self.target_sep = 4.0
        self.spec: FormationSpec = self._make_spec(max(2, cfg.sim_count))
        self.mid = MidpointPose(u=cfg.sequence.hover_alt_m)
        self.origin = self._initial_origin()

        # flight phase
        self.phase = "disarmed"
        self.work_alt = cfg.sequence.work_alt_m
        self.seq_alt = 0.0
        self.seq_hold: dict[int, tuple[float, float]] = {}
        self._phase_t = time.monotonic()

        self.cmd = {"fwd": 0.0, "right": 0.0, "up": 0.0, "yaw": 0.0}
        self.clients: set[web.WebSocketResponse] = set()
        self._roster_sig = None

    # ------------------------------------------------------------------ #
    def _make_spec(self, n: int) -> FormationSpec:
        return geometry.build_formation(self._form["kind"], n, separation_m=self.target_sep,
                                        net=self._form["net"], net_span_m=self._form["net_span"])

    def _initial_origin(self):
        if self.cfg.use_sim:
            return (self.cfg.origin_lat or SIM_ORIGIN[0], self.cfg.origin_lon or SIM_ORIGIN[1],
                    self.cfg.origin_alt or SIM_ORIGIN[2])
        if self.cfg.origin_lat is not None:
            return (self.cfg.origin_lat, self.cfg.origin_lon, self.cfg.origin_alt or 0.0)
        return None

    # -------------------------- discovery ------------------------------ #
    def _on_join(self, e: RosterEntry):
        free = "top free" if e.profile.net_clear_of_props else "props exposed"
        self._notify("info", f"{e.name} online — {e.autopilot.value} {e.frame}, "
                             f"{e.profile.profile_id} ({e.profile.span_m:.2f} m, {free})")

    def _on_leave(self, sysid: int):
        self._notify("warn", f"vehicle {sysid} dropped offline")
        if self.phase in ACTIVE_PHASES and self.discovery.ready_count == 0:
            self.phase = "disarmed"
            self._notify("error", "all vehicles offline — sequence aborted")

    # ------------------------------ loop -------------------------------- #
    async def run(self):
        await self.link.start()
        log.info("link started (%s)", self.link.kind)
        period = 1.0 / self.cfg.loop_hz
        while True:
            try:
                await self._tick(period)
            except Exception:
                log.exception("control tick failed")
            await asyncio.sleep(period)

    async def _tick(self, dt: float):
        snap = self.link.snapshot()
        states = list(snap.values())
        roster = self.discovery.update(snap)

        if self.origin is None:
            for s in states:
                if s.has_fix():
                    self.origin = (s.lat, s.lon, s.alt_msl or 0.0)
                    self._notify("info", f"origin set from {s.name}")
                    break

        # keep the formation sized to the fleet while on the ground
        if self.phase == "disarmed":
            n = max(2, sum(1 for s in states if s.has_fix()))
            if n != len(self.spec.slots):
                self.spec = self._make_spec(n)

        assignment = geometry.assign_slots(self.spec, states)
        await self._advance_phase(dt, states, assignment)
        if self.phase == "airborne":
            self._integrate_input(dt)

        rep = (safety.evaluate(states, self.cfg, self.spec, assignment, self.origin,
                               commanding=self.phase in ACTIVE_PHASES)
               if self.origin else None)

        if self.phase in FORMATION_PHASES and self.origin and rep is not None:
            safety.clamp_separation(self.spec)
            safety.clamp_midpoint(self.mid, self.spec, self.cfg)
            critical = [v for v in rep.blocks if v.code == "too_close"]
            if critical:
                self._notify("warn", "holding — " + critical[0].msg)
            else:
                targets = geometry.targets_for(self.spec, self.mid, self.origin, assignment)
                blocked = {sid for v in rep.blocks for sid in v.sysids}
                for sysid, t in targets.items():
                    if sysid not in blocked:
                        await self.link.send_setpoint(sysid, t)

        if self.origin and rep is not None:
            f = self.cfg.fence
            fence = {"enabled": f.enabled, "center_e": f.center_e, "center_n": f.center_n,
                     "radius_m": f.radius_m, "alt_min_m": f.alt_min_m, "alt_max_m": f.alt_max_m}
            await self._broadcast(state_message(
                states, assignment, self.mid, self.spec, rep, self.origin,
                self.phase == "airborne", fence, self.phase))

        sig = tuple((e.sysid, e.online, e.ready, e.profile.profile_id,
                     e.profile.inverted, e.profile.net_attach) for e in roster)
        if sig != self._roster_sig:
            self._roster_sig = sig
            await self._broadcast(roster_message(roster))

    # --------------------------- phase machine -------------------------- #
    def _avg_alt(self, states):
        a = [s.alt_rel or 0.0 for s in states if s.has_fix()]
        return sum(a) / len(a) if a else 0.0

    def _current_spread(self, snap, assignment) -> float:
        """Average adjacent-slot spacing right now (so spreading starts where the
        drones actually are rather than snapping them together)."""
        slot_pos = {}
        for sysid, slot in assignment.items():
            s = snap.get(sysid)
            if s and s.has_fix():
                slot_pos[slot] = geometry.latlon_to_enu(s.lat, s.lon, s.alt_rel or 0.0, self.origin)
        ds = []
        for i in range(len(self.spec.slots) - 1):
            if i in slot_pos and i + 1 in slot_pos:
                ds.append(math.dist(slot_pos[i][:2], slot_pos[i + 1][:2]))
        return sum(ds) / len(ds) if ds else self.cfg.sequence.start_separation_m

    def _settled(self, snap, assignment, tol=0.7) -> bool:
        pos = geometry.slot_world_positions(self.spec, self.mid)
        worst = 0.0
        for sysid, slot in assignment.items():
            s = snap.get(sysid)
            if s and s.has_fix() and slot < len(pos):
                e, n, _ = geometry.latlon_to_enu(s.lat, s.lon, s.alt_rel or 0.0, self.origin)
                worst = max(worst, math.dist((e, n), pos[slot][:2]))
        return worst < tol

    async def _advance_phase(self, dt, states, assignment):
        sq = self.cfg.sequence
        snap = {s.sysid: s for s in states}
        now = time.monotonic()
        ph = self.phase

        if ph == "arming":
            armed = states and all(s.armed for s in states)
            if armed:
                # ArduCopter only leaves the ground on an explicit GUIDED takeoff;
                # position setpoints alone won't do it (and it auto-disarms ~15 s after
                # arming if no takeoff starts). So command the takeoff now, while armed.
                for s in states:
                    await self.link.takeoff(s.sysid, sq.hover_alt_m)
                self.phase = "ascending"; self._phase_t = now
                self._notify("info", f"taking off to {sq.hover_alt_m:.1f} m hover")
            elif now - self._phase_t > 5.0:
                for s in states:
                    await self.link.arm(s.sysid, False)
                self.phase = "disarmed"; self._phase_t = now
                self._notify("error", "did not arm — check pre-arm (GPS/EKF/safety) and retry")

        elif ph == "ascending":
            # the autopilot flies the climb; we just wait for it to reach hover
            if self._avg_alt(states) >= sq.hover_alt_m - 0.4:
                fix = [s for s in states if s.has_fix()]
                self.mid = geometry.midpoint_of(fix, self.origin); self.mid.u = sq.hover_alt_m
                self.spec.separation_m = max(sq.start_separation_m,
                                             self._current_spread(snap, assignment))
                self.phase = "spreading"; self._phase_t = now
                self._notify("info", f"spreading to {self.target_sep:.1f} m")
            elif now - self._phase_t > 25.0:
                for s in states:
                    await self.link.land(s.sysid)
                self.phase = "landing"; self._phase_t = now
                self._notify("warn", "takeoff did not climb — landing")

        elif ph == "spreading":
            self.mid.u = sq.hover_alt_m
            tgt = self.target_sep
            step = 0.8 * dt
            if self.spec.separation_m < tgt:
                self.spec.separation_m = min(tgt, self.spec.separation_m + step)
            else:
                self.spec.separation_m = max(tgt, self.spec.separation_m - step)
            if abs(self.spec.separation_m - tgt) < 0.1 and self._settled(snap, assignment):
                self.phase = "climbing"; self._phase_t = now
                self._notify("info", f"climbing to {self.work_alt:.1f} m")

        elif ph == "climbing":
            self.mid.u = min(self.work_alt, self.mid.u + sq.climb_rate_ms * dt)
            if self.mid.u >= self.work_alt - 0.05:
                self.phase = "airborne"; self._phase_t = now
                self._notify("info", "airborne — you have the midpoint")

        elif ph == "descending":
            self.mid.u = max(sq.hover_alt_m, self.mid.u - sq.descent_rate_ms * dt)
            if self.mid.u <= sq.hover_alt_m + 0.05:
                for s in states:
                    await self.link.land(s.sysid)
                self.phase = "landing"; self._phase_t = now
                self._notify("info", "landing (vertical descent)")

        elif ph == "landing":
            # LAND mode brings each vehicle straight down and disarms on the ground
            if self._avg_alt(states) <= 0.3 or not any(s.armed for s in states):
                for s in states:
                    await self.link.arm(s.sysid, False)
                self.phase = "disarmed"; self._phase_t = now
                self.cmd = {k: 0.0 for k in self.cmd}
                self._notify("info", "landed and disarmed")
            elif now - self._phase_t > 30.0:
                for s in states:
                    await self.link.arm(s.sysid, False)
                self.phase = "disarmed"; self._phase_t = now

    def _integrate_input(self, dt: float):
        psi = math.radians(self.mid.yaw_deg)
        fwd, right, up, yaw = (self.cmd[k] for k in ("fwd", "right", "up", "yaw"))
        ve = fwd * math.sin(psi) + right * math.cos(psi)
        vn = fwd * math.cos(psi) - right * math.sin(psi)
        self.mid.e += ve * dt
        self.mid.n += vn * dt
        self.mid.u += up * dt
        self.mid.yaw_deg = (self.mid.yaw_deg + yaw * dt) % 360.0

    # --------------------------- commands ------------------------------- #
    async def handle(self, msg: dict):
        t = msg.get("type")
        if t == "input":
            for k in ("fwd", "right", "up", "yaw"):
                if k in msg:
                    self.cmd[k] = float(msg[k])
        elif t == "separation":
            self.target_sep = max(0.5, float(msg.get("value", self.target_sep)))
            if self.phase == "airborne":
                self.spec.separation_m = self.target_sep
        elif t == "set_formation":
            await self._set_formation(msg)
        elif t == "set_profile":
            self.cfg.sysid_profiles[int(msg["sysid"])] = str(msg["profile_id"])
            self._roster_sig = None
        elif t == "command":
            await self._command(msg)

    async def _set_formation(self, msg: dict):
        kind = msg.get("kind", self._form["kind"])
        self._form["kind"] = kind
        self._form["net"] = bool(msg.get("net", self._form["net"]))
        self._form["net_span"] = float(msg.get("net_span", self._form["net_span"]))
        self.target_sep = float(msg.get("separation", self.target_sep))
        n = int(msg.get("n", len(self.spec.slots)))
        if self.phase == "airborne":
            # rebuild keeping the live separation so we don't jump airframes
            live = self.spec.separation_m
            self.spec = self._make_spec(n); self.spec.separation_m = live
        else:
            self.spec = self._make_spec(n)
        self._notify("info", f"formation: {kind} ×{n}, sep {self.target_sep:.1f} m"
                             + (f", net {self._form['net_span']:.1f} m" if self._form["net"] else ""))

    async def _command(self, msg: dict):
        name = msg.get("name")
        snap = self.link.snapshot()
        ids = list(snap.keys())

        if name == "configure":
            await self.reconfigure(msg.get("config", {}))
        elif name == "auto_takeoff":
            ready = [s for s in snap.values() if s.has_fix()]
            if not ready or self.origin is None:
                self._notify("error", "cannot take off — no drone with a GPS fix")
                return
            if self.phase != "disarmed":
                self._notify("warn", "already in a flight sequence")
                return
            self.work_alt = float(msg.get("alt", self.cfg.sequence.work_alt_m))
            for sid in ids:
                await self.link.set_mode(sid, "GUIDED")
                await self.link.arm(sid, True)
            self.phase = "arming"; self._phase_t = time.monotonic()
            self._notify("info", f"auto take-off → {self.work_alt:.0f} m (staged)")
        elif name == "auto_land":
            if self.phase in ("airborne", "climbing", "spreading"):
                self.mid.u = max(self.mid.u, self.cfg.sequence.hover_alt_m)
                self.phase = "descending"; self._phase_t = time.monotonic()
                self.cmd = {k: 0.0 for k in self.cmd}
                self._notify("info", "auto land (staged)")
            elif self.phase == "ascending":
                for sid in ids:
                    await self.link.land(sid)
                self.phase = "landing"; self._phase_t = time.monotonic()
                self._notify("info", "landing")
            else:
                self._notify("warn", "nothing to land")
        elif name == "hold":
            self.cmd = {k: 0.0 for k in self.cmd}
            self._notify("info", "hold — midpoint frozen")
        elif name == "engage":   # manual: jump straight to formation at current state
            ready = [s for s in snap.values() if s.has_fix()]
            if not ready or self.origin is None:
                self._notify("error", "cannot engage — no GPS fix"); return
            self.mid = geometry.midpoint_of(ready, self.origin)
            for sid in ids:
                await self.link.set_mode(sid, "GUIDED")
                await self.link.arm(sid, True)      # must be armed to follow setpoints
            self.phase = "airborne"; self._notify("info", "engaged (manual) — armed, you have the midpoint")
        elif name == "disengage":
            self.cmd = {k: 0.0 for k in self.cmd}
            self._notify("info", "holding position")
        elif name == "arm":
            for sid in ids:
                await self.link.arm(sid, True)
            self._notify("info", "arm sent to all")
        elif name == "disarm":
            self.phase = "disarmed"
            for sid in ids:
                await self.link.arm(sid, False)
            self._notify("warn", "disarm sent to all")

    async def reconfigure(self, c: dict):
        mode = c.get("mode", "sim")
        self.cfg.use_sim = (mode == "sim")
        if "sim_count" in c:
            self.cfg.sim_count = int(c["sim_count"])
        if "connections" in c:
            self.cfg.connections = [s for s in c["connections"] if s.strip()]
        if "names" in c:
            self.cfg.names = {int(k): v for k, v in c["names"].items()}
        if "sysid_profiles" in c:
            self.cfg.sysid_profiles = {int(k): v for k, v in c["sysid_profiles"].items()}
        for p in c.get("profiles", []):
            try:
                prof = VehicleProfile(**p)
                self.cfg.profiles[prof.profile_id] = prof
            except TypeError:
                pass
        if "fence" in c:
            self.cfg.fence = FenceConfig(**{**self.cfg.fence.__dict__, **c["fence"]})
        form = c.get("formation", {})
        self._form["kind"] = form.get("kind", self._form["kind"])
        self._form["net"] = bool(form.get("net", self._form["net"]))
        self._form["net_span"] = float(form.get("net_span", self._form["net_span"]))
        self.target_sep = float(form.get("separation", self.target_sep))

        new = build_link(self.cfg)
        await new.start()
        old, self.link = self.link, new
        try:
            await old.stop()
        except Exception:
            pass

        self.discovery = Discovery(self.cfg, self._on_join, self._on_leave)
        self.origin = self._initial_origin()
        self.spec = self._make_spec(max(2, self.cfg.sim_count if self.cfg.use_sim
                                        else len(self.cfg.connections) or 2))
        self.mid = MidpointPose(u=self.cfg.sequence.hover_alt_m)
        self.phase = "disarmed"
        self.cmd = {k: 0.0 for k in self.cmd}
        self._roster_sig = None
        tgt = "simulator" if self.cfg.use_sim else f"{len(self.cfg.connections)} link(s)"
        self._notify("info", f"reconfigured → {tgt}")
        await self._broadcast(self.config_snapshot())

    def config_snapshot(self) -> dict:
        """Everything the setup screen needs to prefill itself and offer profiles."""
        def prof(p: VehicleProfile) -> dict:
            return {"profile_id": p.profile_id, "name": p.name, "span_m": p.span_m,
                    "mass_kg": p.mass_kg, "max_speed_ms": p.max_speed_ms,
                    "max_accel_ms2": p.max_accel_ms2, "prop_inch": p.prop_inch,
                    "inverted": p.inverted, "net_attach": p.net_attach}
        f = self.cfg.fence
        return {
            "type": "config",
            "mode": "sim" if self.cfg.use_sim else "real",
            "sim_count": self.cfg.sim_count,
            "connections": self.cfg.connections,
            "names": {str(k): v for k, v in self.cfg.names.items()},
            "sysid_profiles": {str(k): v for k, v in self.cfg.sysid_profiles.items()},
            "profiles": [prof(p) for p in self.cfg.profiles.values()],
            "fence": {"enabled": f.enabled, "radius_m": f.radius_m,
                      "alt_min_m": f.alt_min_m, "alt_max_m": f.alt_max_m},
            "formation": {"kind": self._form["kind"], "separation": self.target_sep,
                          "net": self._form["net"], "net_span": self._form["net_span"]},
        }

    # --------------------------- websocket ------------------------------ #
    def _notify(self, level: str, text: str):
        log.info("[%s] %s", level, text)
        asyncio.create_task(self._broadcast(event_message(level, text)))

    async def _broadcast(self, payload: dict):
        if not self.clients:
            return
        data = json.dumps(payload)
        dead = []
        for ws in self.clients:
            try:
                await ws.send_str(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)


# ------------------------------------------------------------------------- #
async def ws_handler(request: web.Request):
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    ctrl: Controller = request.app["controller"]
    ctrl.clients.add(ws)
    ctrl._roster_sig = None       # re-broadcast roster to the new client next tick
    try:
        await ws.send_str(json.dumps(ctrl.config_snapshot()))
    except Exception:
        pass
    try:
        async for raw in ws:
            if raw.type == WSMsgType.TEXT:
                try:
                    msg = parse_command(json.loads(raw.data))
                except json.JSONDecodeError:
                    msg = None
                if msg:
                    await ctrl.handle(msg)
            elif raw.type == WSMsgType.ERROR:
                break
    finally:
        ctrl.clients.discard(ws)
    return ws


async def index(request: web.Request):
    return web.FileResponse(os.path.join(FRONTEND, "index.html"))


def make_app(cfg: Config) -> web.Application:
    app = web.Application()
    ctrl = Controller(cfg)
    app["controller"] = ctrl
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    if os.path.isdir(FRONTEND):
        app.router.add_static("/static/", FRONTEND)

    async def _start(app):
        app["loop_task"] = asyncio.create_task(ctrl.run())

    async def _stop(app):
        app["loop_task"].cancel()
        await ctrl.link.stop()

    app.on_startup.append(_start)
    app.on_cleanup.append(_stop)
    return app
