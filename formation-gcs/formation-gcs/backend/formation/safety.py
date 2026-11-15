"""
Safety layer.

Two jobs:
  1. EVALUATE the fleet and raise violations (warn = caution, block = do not command).
  2. CLAMP setpoints so the operator physically cannot drive the group out of the
     fence, past the net's safe band, or below a floor.

The net band is the interesting one. A net spanning two drones makes them a
constrained pair: open them past the net width and it snaps taut and yanks them off
station; close them too far and the slack cloth sags into the propellers. So a linked
pair is kept inside [slack_min_m, span_m] -- this is the concrete answer to the
"propellers / net entanglement" problem.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .geometry import enu_of
from ..config import Config, FenceConfig
from ..models import FormationSpec, MidpointPose, VehicleProfile, VehicleState

HEARTBEAT_STALE_S = 2.0
SEP_BUFFER_M = 1.0          # extra clearance on top of the two rotor radii
BATT_WARN_PCT = 20.0
BATT_BLOCK_PCT = 8.0


@dataclass
class Violation:
    severity: str              # "warn" | "block"
    code: str
    msg: str
    sysids: list[int] = field(default_factory=list)


@dataclass
class SafetyReport:
    violations: list[Violation] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not any(v.severity == "block" for v in self.violations)

    @property
    def blocks(self) -> list[Violation]:
        return [v for v in self.violations if v.severity == "block"]

    @property
    def warns(self) -> list[Violation]:
        return [v for v in self.violations if v.severity == "warn"]


# --------------------------------------------------------------------------- #
def check_health(state: VehicleState, commanding: bool) -> list[Violation]:
    out: list[Violation] = []
    if commanding and state.heartbeat_age() > HEARTBEAT_STALE_S:
        out.append(Violation("block", "link_lost",
                             f"{state.name}: no heartbeat for {state.heartbeat_age():.0f}s",
                             [state.sysid]))
    if commanding and not state.has_fix():
        out.append(Violation("block", "no_fix",
                             f"{state.name}: no 3D GPS fix", [state.sysid]))
    if 0 <= state.battery_pct < BATT_BLOCK_PCT:
        out.append(Violation("block", "battery_critical",
                             f"{state.name}: battery {state.battery_pct:.0f}%", [state.sysid]))
    elif 0 <= state.battery_pct < BATT_WARN_PCT:
        out.append(Violation("warn", "battery_low",
                             f"{state.name}: battery {state.battery_pct:.0f}%", [state.sysid]))
    return out


def _pairwise_enu(states: list[VehicleState], origin) -> dict[int, tuple[float, float, float]]:
    pos = {}
    for s in states:
        if s.has_fix():
            pos[s.sysid] = enu_of(s, origin)
    return pos


def check_separation(
    states: list[VehicleState],
    profiles: dict[int, VehicleProfile],
    spec: FormationSpec,
    assignment: dict[int, int],
    origin,
) -> list[Violation]:
    out: list[Violation] = []
    pos = _pairwise_enu(states, origin)
    slot_to_sysid = {slot: sysid for sysid, slot in assignment.items()}
    # which (sysid, sysid) pairs are joined by a net?
    net_pairs = {}
    for link in spec.links:
        a, b = slot_to_sysid.get(link.a), slot_to_sysid.get(link.b)
        if a is not None and b is not None:
            net_pairs[frozenset((a, b))] = link

    ids = list(pos.keys())
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a, b = ids[i], ids[j]
            (ea, na, ua), (eb, nb, ub) = pos[a], pos[b]
            d = math.dist((ea, na, ua), (eb, nb, ub))
            collision_min = profiles[a].radius_m + profiles[b].radius_m + SEP_BUFFER_M
            key = frozenset((a, b))
            if key in net_pairs:
                link = net_pairs[key]
                # if either drone carries the net on the same face as its props, the
                # slack cloth can reach the props; if both keep it on the free face,
                # that risk is gone (the inverted / top-free build).
                props_exposed = (not profiles[a].net_clear_of_props) or \
                                (not profiles[b].net_clear_of_props)
                if d > link.span_m:
                    out.append(Violation("warn", "net_taut",
                                         f"net {a}-{b} stretched: {d:.1f}m > {link.span_m:.1f}m span",
                                         [a, b]))
                if d < collision_min:
                    out.append(Violation("block", "too_close",
                                         f"{a}-{b} separation {d:.1f}m < {collision_min:.1f}m min",
                                         [a, b]))
                elif props_exposed and d < link.slack_min_m:
                    out.append(Violation("warn", "net_slack",
                                         f"net {a}-{b} slack into props: {d:.1f}m < {link.slack_min_m:.1f}m",
                                         [a, b]))
            else:
                if d < collision_min:
                    sev = "block" if d < collision_min * 0.6 else "warn"
                    out.append(Violation(sev, "too_close",
                                         f"{a}-{b} separation {d:.1f}m < {collision_min:.1f}m min",
                                         [a, b]))
    return out


def check_fence(states: list[VehicleState], fence: FenceConfig, origin) -> list[Violation]:
    out: list[Violation] = []
    if not fence.enabled:
        return out
    for s in states:
        if not s.has_fix():
            continue
        e, n, u = enu_of(s, origin)
        r = math.dist((e, n), (fence.center_e, fence.center_n))
        if r > fence.radius_m:
            out.append(Violation("block", "fence_breach",
                                 f"{s.name}: {r:.0f}m from centre > {fence.radius_m:.0f}m fence",
                                 [s.sysid]))
        elif r > fence.radius_m * 0.9:
            out.append(Violation("warn", "fence_near",
                                 f"{s.name}: nearing fence ({r:.0f}m)", [s.sysid]))
        if u > fence.alt_max_m or u < fence.alt_min_m:
            out.append(Violation("warn", "alt_band",
                                 f"{s.name}: alt {u:.0f}m outside {fence.alt_min_m:.0f}-{fence.alt_max_m:.0f}m",
                                 [s.sysid]))
    return out


def check_commanded_separation(spec: FormationSpec) -> list[Violation]:
    """Catch an operator opening the formation wider than the net can span -- before
    a single setpoint goes out."""
    out: list[Violation] = []
    for link in spec.links:
        try:
            a, b = spec.slots[link.a], spec.slots[link.b]
        except IndexError:
            continue
        d = math.dist((a.sx * spec.separation_m, a.sy * spec.separation_m),
                      (b.sx * spec.separation_m, b.sy * spec.separation_m))
        if d > link.span_m + 1e-6:
            out.append(Violation("block", "sep_exceeds_net",
                                 f"commanded {d:.1f}m exceeds net span {link.span_m:.1f}m",
                                 []))
    return out


def evaluate(
    states: list[VehicleState],
    cfg: Config,
    spec: FormationSpec,
    assignment: dict[int, int],
    origin,
    commanding: bool = True,
) -> SafetyReport:
    rep = SafetyReport()
    profiles = {s.sysid: cfg.profile_for(s.sysid) for s in states}
    for s in states:
        rep.violations += check_health(s, commanding)
    rep.violations += check_separation(states, profiles, spec, assignment, origin)
    rep.violations += check_fence(states, cfg.fence, origin)
    rep.violations += check_commanded_separation(spec)
    return rep


# --------------------------------------------------------------------------- #
# Clamping -- make unsafe commands unreachable rather than just flagged
# --------------------------------------------------------------------------- #
def clamp_separation(spec: FormationSpec) -> float:
    """Largest separation the net pairs allow; returns the clamped value."""
    cap = spec.separation_m
    for link in spec.links:
        a, b = spec.slots[link.a], spec.slots[link.b]
        unit_d = math.dist((a.sx, a.sy), (b.sx, b.sy))
        if unit_d > 1e-9:
            cap = min(cap, link.span_m / unit_d)
    spec.separation_m = max(0.5, cap)
    return spec.separation_m


def clamp_midpoint(mid: MidpointPose, spec: FormationSpec, cfg: Config) -> MidpointPose:
    """Keep the midpoint -- and the whole formation footprint -- inside the fence and
    altitude band."""
    f = cfg.fence
    if f.enabled:
        mid.u = min(max(mid.u, f.alt_min_m), f.alt_max_m)
        # furthest slot reach so the *edge* drone, not just the centre, stays inside
        reach = max((math.hypot(s.sx, s.sy) for s in spec.slots), default=0.0) * spec.separation_m
        usable = max(0.0, f.radius_m - reach)
        de, dn = mid.e - f.center_e, mid.n - f.center_n
        r = math.hypot(de, dn)
        if r > usable and r > 1e-6:
            k = usable / r
            mid.e = f.center_e + de * k
            mid.n = f.center_n + dn * k
    else:
        mid.u = max(0.0, mid.u)
    return mid
