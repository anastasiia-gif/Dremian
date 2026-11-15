"""
Websocket wire protocol.

Server -> client
  roster   discovered fleet (identity, profile, readiness) -- sent on change
  state    live telemetry + midpoint + formation + safety -- sent every loop tick
  event    a one-line log/notice {level, msg}

Client -> server
  input         {de, dn, du, dyaw}            nudge the midpoint (operator stick)
  set_formation {kind, n, separation, net, net_span}
  separation    {value}
  command       {name: engage|disengage|arm|disarm|takeoff|land, alt?}
  set_profile   {sysid, profile_id}

Names describe what the operator controls, not how it's wired underneath.
"""
from __future__ import annotations

import time

from .discovery import RosterEntry
from .formation.geometry import enu_of
from .formation.safety import SafetyReport
from .models import FormationSpec, MidpointPose, VehicleState


def roster_entry_dict(e: RosterEntry) -> dict:
    return {
        "sysid": e.sysid,
        "name": e.name,
        "autopilot": e.autopilot.value,
        "frame": e.frame,
        "fw": e.fw_version,
        "profile": {"id": e.profile.profile_id, "name": e.profile.name,
                    "span_m": e.profile.span_m, "mass_kg": e.profile.mass_kg,
                    "inverted": e.profile.inverted, "net_attach": e.profile.net_attach,
                    "props_clear": e.profile.net_clear_of_props},
        "online": e.online,
        "fix": e.has_fix,
        "armed": e.armed,
        "mode": e.mode,
        "battery": round(e.battery_pct, 1),
        "guided": e.supports_guided,
        "ready": e.ready,
    }


def roster_message(roster: list[RosterEntry]) -> dict:
    return {"type": "roster", "ts": time.time(),
            "vehicles": [roster_entry_dict(e) for e in roster]}


def vehicle_state_dict(st: VehicleState, origin, slot: int | None) -> dict:
    e = n = u = None
    if st.has_fix() and origin:
        e, n, u = enu_of(st, origin)
    return {
        "sysid": st.sysid, "name": st.name, "slot": slot,
        "lat": st.lat, "lon": st.lon, "alt": st.alt_rel,
        "e": e, "n": n, "u": u,
        "heading": round(st.heading, 1),
        "armed": st.armed, "mode": st.mode,
        "battery": round(st.battery_pct, 1),
        "online": st.heartbeat_age() < 3.0,
    }


def safety_dict(rep: SafetyReport) -> dict:
    return {
        "ok": rep.ok,
        "violations": [
            {"severity": v.severity, "code": v.code, "msg": v.msg, "sysids": v.sysids}
            for v in rep.violations
        ],
    }


def state_message(states: list[VehicleState], assignment: dict[int, int],
                  mid: MidpointPose, spec: FormationSpec, rep: SafetyReport,
                  origin, engaged: bool, fence: dict | None = None,
                  phase: str = "disarmed") -> dict:
    return {
        "type": "state",
        "ts": time.time(),
        "engaged": engaged,
        "phase": phase,
        "fence": fence,
        "origin": {"lat": origin[0], "lon": origin[1], "alt": origin[2]} if origin else None,
        "midpoint": {"e": round(mid.e, 2), "n": round(mid.n, 2),
                     "u": round(mid.u, 2), "yaw": round(mid.yaw_deg, 1)},
        "formation": {"name": spec.name, "separation": round(spec.separation_m, 2),
                      "slots": len(spec.slots), "nets": len(spec.links),
                      "net_span": round(spec.links[0].span_m, 2) if spec.links else None},
        "vehicles": [vehicle_state_dict(s, origin, assignment.get(s.sysid)) for s in states],
        "safety": safety_dict(rep),
    }


def event_message(level: str, msg: str) -> dict:
    return {"type": "event", "ts": time.time(), "level": level, "msg": msg}


# --------------------------------------------------------------------------- #
def parse_command(raw: dict) -> dict | None:
    """Light validation of an inbound client message; returns it or None if junk."""
    if not isinstance(raw, dict) or "type" not in raw:
        return None
    t = raw["type"]
    known = {"input", "set_formation", "separation", "command", "set_profile"}
    return raw if t in known else None
