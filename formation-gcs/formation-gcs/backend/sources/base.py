"""
The link abstraction.

Everything above this line (geometry, safety, web app) talks to a VehicleLink and
never to pymavlink or a socket directly. Swap SimLink for MAVLinkLink -- or add a PX4
link later -- without touching anything else. This is what makes the stack
"one architecture, many vehicles" instead of hard-wired to two ArduPilot drones.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from ..models import VehicleState, WorldTarget


class VehicleLink(ABC):
    kind: str = "base"

    @abstractmethod
    async def start(self) -> None:
        """Open the transport and begin ingesting telemetry."""

    @abstractmethod
    async def stop(self) -> None:
        """Close cleanly."""

    @abstractmethod
    def snapshot(self) -> dict[int, VehicleState]:
        """Current state of every known vehicle, keyed by sysid (copies, safe to read)."""

    @abstractmethod
    async def send_setpoint(self, sysid: int, target: WorldTarget) -> None:
        """Command one vehicle to a global guided position target."""

    # Commands below have safe no-op defaults so a minimal link still works.
    async def set_mode(self, sysid: int, mode: str) -> None:  # pragma: no cover
        ...

    async def arm(self, sysid: int, arm: bool = True) -> None:  # pragma: no cover
        ...

    async def takeoff(self, sysid: int, alt_m: float) -> None:  # pragma: no cover
        ...

    async def land(self, sysid: int) -> None:  # pragma: no cover
        ...
