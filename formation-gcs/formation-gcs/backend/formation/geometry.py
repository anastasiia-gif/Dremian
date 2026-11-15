"""
Formation geometry.

The operator flies a single virtual point -- the formation midpoint -- with a pose
(east, north, up, yaw). Every drone's target is derived from that pose plus its slot
in the formation, so the whole group turns, climbs and translates as one rigid body:
the "stick with a controllable middle" idea, generalised to N drones and any shape.

Frames
------
* World ENU: metres east / north / up of a fixed origin (a local tangent plane).
* Formation-local: x forward (along yaw), y left, z up. Slot coordinates are in
  *unit* spacing and are multiplied by separation_m, so a shape opens and closes
  smoothly with one knob.

Global setpoints (lat/lon) are produced with an equirectangular projection about the
origin -- accurate to well under a metre over the tens-of-metres a formation spans.
"""
from __future__ import annotations

import math

from ..models import FormationSpec, MidpointPose, NetLink, Slot, VehicleState, WorldTarget

EARTH_R = 6_371_000.0  # m


# --------------------------------------------------------------------------- #
# Local tangent plane <-> WGS84
# --------------------------------------------------------------------------- #
def enu_to_latlon(e: float, n: float, u: float, origin: tuple[float, float, float]):
    lat0, lon0, alt0 = origin
    dlat = math.degrees(n / EARTH_R)
    dlon = math.degrees(e / (EARTH_R * math.cos(math.radians(lat0))))
    return lat0 + dlat, lon0 + dlon, alt0 + u


def latlon_to_enu(lat: float, lon: float, alt: float, origin: tuple[float, float, float]):
    lat0, lon0, alt0 = origin
    e = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    n = math.radians(lat - lat0) * EARTH_R
    return e, n, alt - alt0


def enu_of(v, origin: tuple[float, float, float]):
    """A vehicle's local ENU position.

    Horizontal comes from lat/lon; the up axis is the vehicle's *relative* altitude
    (height above home), which IS the ENU up coordinate -- so it is used directly and
    must NOT be re-differenced against the origin's MSL. Feeding alt_rel into
    latlon_to_enu (which subtracts origin MSL) is the classic bug that puts drones a few
    hundred metres underground.
    """
    e, n, _ = latlon_to_enu(v.lat, v.lon, origin[2], origin)
    return e, n, (v.alt_rel or 0.0)


# --------------------------------------------------------------------------- #
# Formation factory
# --------------------------------------------------------------------------- #
def build_formation(
    kind: str,
    n: int,
    separation_m: float = 4.0,
    *,
    net: bool = False,
    net_span_m: float | None = None,
    net_slack_min_m: float = 0.0,
    rows: int | None = None,
) -> FormationSpec:
    """Build a preset shape for `n` drones, centred on the midpoint.

    kind: "line"   side-by-side, perpendicular to travel (good for a catching net)
          "column" nose-to-tail, along travel
          "vee"    a V opening forward
          "grid"   rows x cols block
    net:  add a net link between each adjacent slot (the catching cloth). The net's
          physical width is net_span_m (defaults to the current separation).
    """
    kind = kind.lower()
    slots: list[Slot] = []

    def centred(i: int, count: int) -> float:
        return i - (count - 1) / 2.0

    if kind == "line":
        for i in range(n):
            slots.append(Slot(sx=0.0, sy=centred(i, n), label=f"S{i}"))
    elif kind == "column":
        for i in range(n):
            slots.append(Slot(sx=-centred(i, n), sy=0.0, label=f"S{i}"))
    elif kind == "vee":
        # leader at front centre, wings sweep back-left and back-right
        slots.append(Slot(sx=centred(0, 1), sy=0.0, label="lead"))
        per_wing = (n - 1)
        for k in range(per_wing):
            side = -1 if k % 2 == 0 else 1
            depth = (k // 2) + 1
            slots.append(Slot(sx=-depth, sy=side * depth, label=f"w{k}"))
        slots = slots[:n]
    elif kind == "grid":
        r = rows or max(1, int(round(math.sqrt(n))))
        c = math.ceil(n / r)
        idx = 0
        for ri in range(r):
            for ci in range(c):
                if idx >= n:
                    break
                slots.append(Slot(sx=centred(ri, r), sy=centred(ci, c), label=f"r{ri}c{ci}"))
                idx += 1
    else:
        raise ValueError(f"unknown formation kind: {kind!r}")

    links: list[NetLink] = []
    if net and len(slots) >= 2:
        span = net_span_m if net_span_m is not None else separation_m
        for a in range(len(slots) - 1):
            links.append(NetLink(a=a, b=a + 1, span_m=span, slack_min_m=net_slack_min_m))

    return FormationSpec(name=f"{kind}{n}", slots=slots, links=links, separation_m=separation_m)


# --------------------------------------------------------------------------- #
# Midpoint -> per-slot world positions / targets
# --------------------------------------------------------------------------- #
def slot_world_positions(spec: FormationSpec, mid: MidpointPose) -> list[tuple[float, float, float]]:
    """Return each slot's target position in world ENU (metres)."""
    psi = math.radians(mid.yaw_deg)
    sin_p, cos_p = math.sin(psi), math.cos(psi)
    sep = spec.separation_m
    out: list[tuple[float, float, float]] = []
    for s in spec.slots:
        # forward unit = (sin psi, cos psi); left unit = (-cos psi, sin psi)
        off_e = sep * (s.sx * sin_p - s.sy * cos_p)
        off_n = sep * (s.sx * cos_p + s.sy * sin_p)
        out.append((mid.e + off_e, mid.n + off_n, mid.u + s.sz))
    return out


def assign_slots(spec: FormationSpec, vehicles: list[VehicleState]) -> dict[int, int]:
    """Map sysid -> slot index. Deterministic: drones sorted by sysid fill slots in
    order, so the operator always knows which airframe sits where."""
    ordered = sorted(v.sysid for v in vehicles)
    return {sysid: i for i, sysid in enumerate(ordered[: len(spec.slots)])}


def targets_for(
    spec: FormationSpec,
    mid: MidpointPose,
    origin: tuple[float, float, float],
    assignment: dict[int, int],
) -> dict[int, WorldTarget]:
    """Compute the global guided setpoint for every assigned vehicle."""
    positions = slot_world_positions(spec, mid)
    targets: dict[int, WorldTarget] = {}
    for sysid, slot_idx in assignment.items():
        if slot_idx >= len(positions):
            continue
        e, n, u = positions[slot_idx]
        lat, lon, alt_rel = enu_to_latlon(e, n, u, origin)
        targets[sysid] = WorldTarget(lat=lat, lon=lon, alt_rel=u, yaw_deg=mid.yaw_deg)
    return targets


def midpoint_of(vehicles: list[VehicleState], origin: tuple[float, float, float]) -> MidpointPose:
    """Estimate the current midpoint from where the drones actually are -- used to
    seed the operator's control point so engaging formation doesn't jump them."""
    pts = []
    for v in vehicles:
        if v.has_fix():
            e, n, u = enu_of(v, origin)
            pts.append((e, n, u, v.heading))
    if not pts:
        return MidpointPose()
    e = sum(p[0] for p in pts) / len(pts)
    n = sum(p[1] for p in pts) / len(pts)
    u = sum(p[2] for p in pts) / len(pts)
    # circular mean of headings
    sy = sum(math.sin(math.radians(p[3])) for p in pts)
    cy = sum(math.cos(math.radians(p[3])) for p in pts)
    yaw = math.degrees(math.atan2(sy, cy)) % 360.0
    return MidpointPose(e=e, n=n, u=max(0.0, u), yaw_deg=yaw)
