"""
Discovery -- Project A's heart.

Watches whatever the link reports and turns a bag of source-system ids into a named,
profiled, readiness-checked fleet roster. This is where "auto-connect and read the
drones" actually lives: identity and capabilities come from the autopilot (HEARTBEAT /
AUTOPILOT_VERSION); the physical profile (span, mass) -- which the autopilot does not
know -- is matched in from config. Departures and arrivals fire callbacks so the UI
and the control loop react without polling.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

from .config import Config
from .models import Autopilot, VehicleProfile, VehicleState

CAP_SETPOS_GLOBAL = 0x100  # MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT
ONLINE_TIMEOUT_S = 3.0


@dataclass
class RosterEntry:
    sysid: int
    name: str
    autopilot: Autopilot
    frame: str
    fw_version: str
    profile: VehicleProfile
    online: bool
    has_fix: bool
    armed: bool
    mode: str
    battery_pct: float
    supports_guided: bool

    @property
    def ready(self) -> bool:
        return self.online and self.has_fix and self.supports_guided


class Discovery:
    def __init__(self, cfg: Config,
                 on_join: Callable[[RosterEntry], None] | None = None,
                 on_leave: Callable[[int], None] | None = None):
        self.cfg = cfg
        self.on_join = on_join
        self.on_leave = on_leave
        self._known: set[int] = set()

    def _supports_guided(self, st: VehicleState) -> bool:
        if st.capabilities & CAP_SETPOS_GLOBAL:
            return True
        # autopilots we know do guided even before AUTOPILOT_VERSION arrives
        return st.autopilot in (Autopilot.ARDUPILOT, Autopilot.PX4, Autopilot.SIM)

    def entry_for(self, st: VehicleState, now: float) -> RosterEntry:
        prof = self.cfg.profile_for(st.sysid)
        return RosterEntry(
            sysid=st.sysid,
            name=st.name or self.cfg.name_for(st.sysid),
            autopilot=st.autopilot,
            frame=st.frame.value if hasattr(st.frame, "value") else str(st.frame),
            fw_version=st.fw_version,
            profile=prof,
            online=st.heartbeat_age(now) < ONLINE_TIMEOUT_S,
            has_fix=st.has_fix(),
            armed=st.armed,
            mode=st.mode,
            battery_pct=st.battery_pct,
            supports_guided=self._supports_guided(st),
        )

    def update(self, snapshot: dict[int, VehicleState]) -> list[RosterEntry]:
        """Recompute the roster and fire join/leave callbacks. Returns current roster."""
        now = time.monotonic()
        roster: list[RosterEntry] = []
        seen_online: set[int] = set()

        for sysid in sorted(snapshot):
            entry = self.entry_for(snapshot[sysid], now)
            roster.append(entry)
            if entry.online:
                seen_online.add(sysid)
                if sysid not in self._known:
                    self._known.add(sysid)
                    if self.on_join:
                        self.on_join(entry)

        for sysid in list(self._known):
            if sysid not in seen_online:
                self._known.discard(sysid)
                if self.on_leave:
                    self.on_leave(sysid)

        return roster

    @property
    def ready_count(self) -> int:
        return len(self._known)
