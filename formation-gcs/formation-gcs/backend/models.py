"""
Core domain models for formation-gcs.

These are transport-agnostic: a SimLink and a MAVLinkLink both populate the same
VehicleState, so everything above the link layer (geometry, safety, the web app)
is written once and works for either.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field, replace
from enum import Enum


class Autopilot(str, Enum):
    ARDUPILOT = "ardupilot"
    PX4 = "px4"
    GENERIC = "generic"
    SIM = "sim"
    UNKNOWN = "unknown"


class FrameType(str, Enum):
    QUAD = "quad"
    HEXA = "hexa"
    OCTO = "octo"
    PLANE = "plane"
    VTOL = "vtol"
    UNKNOWN = "unknown"


@dataclass
class VehicleProfile:
    """Static physical facts the flight controller does NOT store.

    An ArduPilot FC knows its frame *class* (quad/hexa) and its tuning, but it has
    no idea of the vehicle's real rotor span or mass -- that lives nowhere on the
    autopilot. So "auto-read drone size" is impossible from the FC alone; instead
    each drone is matched to a profile_id, and the real numbers come from here.
    """
    profile_id: str = "default"
    name: str = "Generic quad"
    span_m: float = 0.50          # rotor tip to rotor tip = collision diameter
    mass_kg: float = 1.20
    max_speed_ms: float = 8.0
    max_accel_ms2: float = 4.0
    prop_inch: float = 8.0
    # --- net/prop geometry (the catching rig) ---
    inverted: bool = False        # props-down build: the top deck is left free
    net_attach: str = "bottom"    # which side the cloth mounts to: "top" | "bottom"
    notes: str = ""

    @property
    def radius_m(self) -> float:
        return self.span_m / 2.0

    @property
    def props_side(self) -> str:
        # a normal multirotor carries its props up top; inverted flips them under
        return "bottom" if self.inverted else "top"

    @property
    def net_clear_of_props(self) -> bool:
        # if the cloth sits on the opposite face from the props, slack can't reach them
        return self.net_attach != self.props_side


@dataclass
class VehicleState:
    """Live, per-vehicle state. Source of truth for position is global (lat/lon);
    the app projects it into a shared local ENU frame for geometry."""
    sysid: int
    name: str = ""

    # --- discovered identity / capabilities ---
    autopilot: Autopilot = Autopilot.UNKNOWN
    frame: FrameType = FrameType.UNKNOWN
    fw_version: str = ""
    capabilities: int = 0
    profile_id: str = "default"

    # --- live telemetry ---
    lat: float | None = None       # deg
    lon: float | None = None       # deg
    alt_msl: float | None = None   # m above mean sea level
    alt_rel: float | None = None   # m above home/origin
    vx: float = 0.0                # ENU east  m/s
    vy: float = 0.0                # ENU north m/s
    vz: float = 0.0                # up        m/s
    heading: float = 0.0           # deg, compass (0 = North, clockwise)

    # --- status ---
    armed: bool = False
    mode: str = ""
    fix_type: int = 0              # 0-1 none, 2 = 2D, 3+ = 3D
    num_sats: int = 0
    battery_v: float = 0.0
    battery_pct: float = -1.0      # -1 = unknown

    # --- liveness (monotonic seconds) ---
    last_heartbeat: float = 0.0
    last_position: float = 0.0

    def heartbeat_age(self, now: float | None = None) -> float:
        now = time.monotonic() if now is None else now
        return (now - self.last_heartbeat) if self.last_heartbeat else 1e9

    def has_fix(self) -> bool:
        return self.fix_type >= 3 and self.lat is not None

    def copy(self) -> "VehicleState":
        return replace(self)


@dataclass
class WorldTarget:
    """A global guided setpoint for one vehicle (ArduPilot GUIDED / PX4 OFFBOARD)."""
    lat: float
    lon: float
    alt_rel: float                 # m above origin/home
    yaw_deg: float | None = None
    # optional ENU velocity feed-forward (m/s) -- improves tracking in fast turns
    vx: float | None = None
    vy: float | None = None
    vz: float | None = None


# --------------------------------------------------------------------------- #
# Formation description
# --------------------------------------------------------------------------- #

@dataclass
class Slot:
    """A station in the formation's local frame.

    Frame: x = forward (along the formation heading), y = left, z = up.
    Coordinates are in *unit* spacing and get multiplied by separation_m, so the
    same shape scales smoothly as the operator opens or closes the formation.
    """
    sx: float
    sy: float
    sz: float = 0.0
    label: str = ""


@dataclass
class NetLink:
    """A net/cloth spanning two slots -- the thing that actually catches objects.

    It turns a pair of free drones into a constrained pair: too far apart and the
    net snaps taut and yanks them; too close and it droops into the props. The
    safety layer keeps every linked pair inside [slack_min_m, span_m]."""
    a: int                         # slot index
    b: int                         # slot index
    span_m: float                  # net width when taut (hard upper bound)
    slack_min_m: float = 0.0       # min separation before slack net reaches props


@dataclass
class FormationSpec:
    name: str
    slots: list[Slot]
    links: list[NetLink] = field(default_factory=list)
    separation_m: float = 4.0      # the scale knob the operator drives


@dataclass
class MidpointPose:
    """World pose of the formation's virtual centre -- the 'stick midpoint' the
    operator flies. Everything else is derived from this plus the FormationSpec."""
    e: float = 0.0                 # m east of origin
    n: float = 0.0                 # m north of origin
    u: float = 5.0                 # m up  (target altitude above origin)
    yaw_deg: float = 0.0
