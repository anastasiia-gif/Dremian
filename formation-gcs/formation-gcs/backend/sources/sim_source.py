"""
SimLink -- a hardware-free vehicle link.

Models each drone as a point mass that chases its guided setpoint under its profile's
speed and acceleration limits, and reports exactly the telemetry a real link would
(lat/lon, heading, fix, battery). It exists so the formation logic, safety layer and
UI can be exercised end-to-end on a laptop with zero radios, zero SITL, zero risk --
the step the original prototype kept tripping over before flights.
"""
from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass

from .base import VehicleLink
from ..formation.geometry import enu_to_latlon, latlon_to_enu
from ..models import Autopilot, FrameType, VehicleProfile, VehicleState, WorldTarget

PHYS_HZ = 50.0
KP_POS = 1.2            # position -> desired-velocity gain
MAX_YAW_RATE = 90.0    # deg/s


@dataclass
class _Drone:
    sysid: int
    name: str
    profile: VehicleProfile
    e: float = 0.0
    n: float = 0.0
    u: float = 0.0
    ve: float = 0.0
    vn: float = 0.0
    vu: float = 0.0
    heading: float = 0.0
    armed: bool = False
    mode: str = "STABILIZE"
    battery_pct: float = 100.0
    target: WorldTarget | None = None


class SimLink(VehicleLink):
    kind = "sim"

    def __init__(self, count: int, origin: tuple[float, float, float],
                 profiles: list[VehicleProfile] | None = None,
                 names: dict[int, str] | None = None):
        self.origin = origin
        names = names or {}
        self.drones: dict[int, _Drone] = {}
        for i in range(count):
            sysid = i + 1
            prof = (profiles[i] if profiles and i < len(profiles)
                    else VehicleProfile())
            # start spread out in a short line so they have distinct positions
            d = _Drone(sysid=sysid, name=names.get(sysid, f"SIM{sysid}"), profile=prof,
                       e=(i - (count - 1) / 2) * 3.0, n=0.0, u=0.0)
            self.drones[sysid] = d
        self._task: asyncio.Task | None = None
        self._running = False
        self._last = time.monotonic()

    # ------------------------------------------------------------------ #
    async def start(self) -> None:
        self._running = True
        self._last = time.monotonic()
        self._task = asyncio.create_task(self._loop())

    async def stop(self) -> None:
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    async def _loop(self) -> None:
        dt = 1.0 / PHYS_HZ
        while self._running:
            now = time.monotonic()
            step = min(0.1, now - self._last)
            self._last = now
            for d in self.drones.values():
                self._step(d, step)
            await asyncio.sleep(dt)

    def _step(self, d: _Drone, dt: float) -> None:
        if dt <= 0:
            return
        if d.armed:
            d.battery_pct = max(0.0, d.battery_pct - 0.04 * dt)
        if not (d.armed and d.target):
            # bleed velocity to a stop when not actively guided
            d.ve *= 0.9; d.vn *= 0.9; d.vu *= 0.9
            d.e += d.ve * dt; d.n += d.vn * dt; d.u = max(0.0, d.u + d.vu * dt)
            return

        te, tn, _ = latlon_to_enu(d.target.lat, d.target.lon, 0.0, self.origin)
        tu = d.target.alt_rel          # alt_rel is already 'up above origin'
        ex, ey, ez = te - d.e, tn - d.n, tu - d.u
        dist = math.sqrt(ex * ex + ey * ey + ez * ez)
        vmax, amax = d.profile.max_speed_ms, d.profile.max_accel_ms2
        if dist > 1e-3:
            speed = min(vmax, KP_POS * dist)
            dve = (ex / dist) * speed - d.ve
            dvn = (ey / dist) * speed - d.vn
            dvu = (ez / dist) * speed - d.vu
            # limit acceleration magnitude
            dv = math.sqrt(dve * dve + dvn * dvn + dvu * dvu)
            cap = amax * dt
            if dv > cap and dv > 1e-9:
                k = cap / dv
                dve *= k; dvn *= k; dvu *= k
            d.ve += dve; d.vn += dvn; d.vu += dvu
        else:
            d.ve *= 0.6; d.vn *= 0.6; d.vu *= 0.6

        d.e += d.ve * dt; d.n += d.vn * dt; d.u = max(0.0, d.u + d.vu * dt)

        # yaw toward commanded heading
        if d.target.yaw_deg is not None:
            err = ((d.target.yaw_deg - d.heading + 180) % 360) - 180
            step = max(-MAX_YAW_RATE * dt, min(MAX_YAW_RATE * dt, err))
            d.heading = (d.heading + step) % 360

    # ------------------------------------------------------------------ #
    def snapshot(self) -> dict[int, VehicleState]:
        now = time.monotonic()
        out: dict[int, VehicleState] = {}
        for d in self.drones.values():
            lat, lon, alt = enu_to_latlon(d.e, d.n, d.u, self.origin)
            out[d.sysid] = VehicleState(
                sysid=d.sysid, name=d.name,
                autopilot=Autopilot.SIM, frame=FrameType.QUAD,
                fw_version="SimLink 1.0", profile_id=d.profile.profile_id,
                lat=lat, lon=lon, alt_msl=alt, alt_rel=d.u,
                vx=d.ve, vy=d.vn, vz=d.vu, heading=d.heading,
                armed=d.armed, mode=d.mode, fix_type=3, num_sats=14,
                battery_v=11.1 * (d.battery_pct / 100.0) + 10.0, battery_pct=d.battery_pct,
                last_heartbeat=now, last_position=now,
            )
        return out

    async def send_setpoint(self, sysid: int, target: WorldTarget) -> None:
        if sysid in self.drones:
            self.drones[sysid].target = target

    async def set_mode(self, sysid: int, mode: str) -> None:
        if sysid in self.drones:
            self.drones[sysid].mode = mode

    async def arm(self, sysid: int, arm: bool = True) -> None:
        if sysid in self.drones:
            self.drones[sysid].armed = arm

    async def takeoff(self, sysid: int, alt_m: float) -> None:
        d = self.drones.get(sysid)
        if d:
            d.armed = True
            d.mode = "GUIDED"
            lat, lon, _ = enu_to_latlon(d.e, d.n, alt_m, self.origin)
            d.target = WorldTarget(lat=lat, lon=lon, alt_rel=alt_m, yaw_deg=d.heading)

    async def land(self, sysid: int) -> None:
        d = self.drones.get(sysid)
        if d:
            lat, lon, _ = enu_to_latlon(d.e, d.n, 0.0, self.origin)
            d.target = WorldTarget(lat=lat, lon=lon, alt_rel=0.0, yaw_deg=d.heading)
