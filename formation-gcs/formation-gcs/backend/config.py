"""
Configuration for formation-gcs.

Loads from a JSON file (default: config.json next to run.py) and lets environment
variables override the few things you tweak in the field, so you can change a port
or a connection string without editing the file.

    FORMATION_CONNS     comma-separated connection strings
    FORMATION_WS_HOST   websocket/http bind host
    FORMATION_WS_PORT   websocket/http bind port
    FORMATION_SIM       "1" to force the simulator regardless of the file
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field

from .models import VehicleProfile


# A small built-in library so a fresh install has something to match against.
BUILTIN_PROFILES: dict[str, VehicleProfile] = {
    "default": VehicleProfile("default", "Generic quad", span_m=0.50, mass_kg=1.2),
    "5inch":   VehicleProfile("5inch", "5\" freestyle", span_m=0.25, mass_kg=0.6,
                              max_speed_ms=20, max_accel_ms2=15, prop_inch=5),
    "8inch":   VehicleProfile("8inch", "8\" long-range", span_m=0.42, mass_kg=1.1,
                              max_speed_ms=14, max_accel_ms2=8, prop_inch=8),
    "10inch":  VehicleProfile("10inch", "10\" heavy lift", span_m=0.62, mass_kg=2.4,
                              max_speed_ms=10, max_accel_ms2=5, prop_inch=10),
}


@dataclass
class FenceConfig:
    enabled: bool = True
    center_e: float = 0.0          # m east of origin
    center_n: float = 0.0          # m north of origin
    radius_m: float = 80.0
    alt_min_m: float = 1.0
    alt_max_m: float = 40.0


@dataclass
class SequenceConfig:
    """Staged take-off / landing so a netted group lifts and settles evenly."""
    hover_alt_m: float = 2.5       # low hover where spreading/closing happens
    work_alt_m: float = 10.0       # default working altitude after auto take-off
    ascent_rate_ms: float = 0.6    # gentle, so the net lifts without yanking
    climb_rate_ms: float = 1.0     # hover -> working altitude
    descent_rate_ms: float = 0.5   # working/hover -> ground
    settle_s: float = 1.5          # idle on the ground before disarm
    start_separation_m: float = 1.5  # spacing on the ground before spreading


@dataclass
class Config:
    # links
    use_sim: bool = True
    sim_count: int = 2
    connections: list[str] = field(default_factory=list)

    # web
    ws_host: str = "127.0.0.1"
    ws_port: int = 8765

    # control loop
    loop_hz: float = 10.0

    # shared local frame origin (None -> first GPS fix becomes origin)
    origin_lat: float | None = None
    origin_lon: float | None = None
    origin_alt: float | None = None

    # per-sysid metadata
    names: dict[int, str] = field(default_factory=dict)            # sysid -> name
    sysid_profiles: dict[int, str] = field(default_factory=dict)   # sysid -> profile_id

    # libraries
    profiles: dict[str, VehicleProfile] = field(default_factory=lambda: dict(BUILTIN_PROFILES))
    fence: FenceConfig = field(default_factory=FenceConfig)
    sequence: SequenceConfig = field(default_factory=SequenceConfig)

    # ------------------------------------------------------------------ #
    def profile_for(self, sysid: int) -> VehicleProfile:
        pid = self.sysid_profiles.get(sysid, "default")
        return self.profiles.get(pid, self.profiles["default"])

    def name_for(self, sysid: int) -> str:
        return self.names.get(sysid, f"D{sysid}")


def _int_keys(d: dict) -> dict[int, str]:
    return {int(k): v for k, v in (d or {}).items()}


def load_config(path: str | None = None) -> Config:
    cfg = Config()

    # ---- file ----
    if path and os.path.exists(path):
        with open(path) as fh:
            raw = json.load(fh)

        cfg.use_sim = raw.get("use_sim", cfg.use_sim)
        cfg.sim_count = raw.get("sim_count", cfg.sim_count)
        cfg.connections = raw.get("connections", cfg.connections)
        cfg.ws_host = raw.get("ws_host", cfg.ws_host)
        cfg.ws_port = raw.get("ws_port", cfg.ws_port)
        cfg.loop_hz = raw.get("loop_hz", cfg.loop_hz)

        origin = raw.get("origin") or {}
        cfg.origin_lat = origin.get("lat", cfg.origin_lat)
        cfg.origin_lon = origin.get("lon", cfg.origin_lon)
        cfg.origin_alt = origin.get("alt", cfg.origin_alt)

        cfg.names = _int_keys(raw.get("names", {}))
        cfg.sysid_profiles = _int_keys(raw.get("sysid_profiles", {}))

        for p in raw.get("profiles", []):
            prof = VehicleProfile(**p)
            cfg.profiles[prof.profile_id] = prof

        f = raw.get("fence") or {}
        cfg.fence = FenceConfig(**{**cfg.fence.__dict__, **f})

        sq = raw.get("sequence") or {}
        cfg.sequence = SequenceConfig(**{**cfg.sequence.__dict__, **sq})

    # ---- env overrides ----
    if os.environ.get("FORMATION_CONNS"):
        cfg.connections = [c.strip() for c in os.environ["FORMATION_CONNS"].split(",") if c.strip()]
        cfg.use_sim = False
    if os.environ.get("FORMATION_WS_HOST"):
        cfg.ws_host = os.environ["FORMATION_WS_HOST"]
    if os.environ.get("FORMATION_WS_PORT"):
        cfg.ws_port = int(os.environ["FORMATION_WS_PORT"])
    if os.environ.get("FORMATION_SIM") == "1":
        cfg.use_sim = True

    return cfg
