"""
MAVLinkLink -- the real link, over pymavlink.

Handles the things the original hard-wired prototype assumed away:
  * any number of vehicles, demultiplexed by source system id;
  * one connection carrying several drones, or several connections each carrying one;
  * discovery from HEARTBEAT + AUTOPILOT_VERSION (autopilot, frame, capabilities, fw).

pymavlink is blocking, so each connection gets a daemon reader thread that updates a
shared state table; the async side only ever reads snapshots and pushes setpoints.
The import is lazy on purpose -- sim mode must never require pymavlink to be installed.

NOTE: this path needs SITL or real radios to exercise; the simulator is the unit-tested
one. See docs/ARCHITECTURE.md for the bring-up checklist.
"""
from __future__ import annotations

import threading
import time

from .base import VehicleLink
from ..models import Autopilot, FrameType, VehicleState, WorldTarget

_AP_MAP = {0: Autopilot.GENERIC, 3: Autopilot.ARDUPILOT, 12: Autopilot.PX4}
_FRAME_MAP = {1: FrameType.PLANE, 2: FrameType.QUAD, 13: FrameType.HEXA,
              14: FrameType.OCTO, 20: FrameType.VTOL}


class MAVLinkLink(VehicleLink):
    kind = "mavlink"

    def __init__(self, connections: list[str], names: dict[int, str] | None = None,
                 sysid_profiles: dict[int, str] | None = None,
                 source_system: int = 255, stream_rate_hz: int = 5):
        from pymavlink import mavutil  # lazy import
        self.mavutil = mavutil
        self.conn_strings = connections
        self.names = names or {}
        self.sysid_profiles = sysid_profiles or {}
        self.source_system = source_system
        self.stream_rate_hz = stream_rate_hz

        self._conns: list = []
        self._locks: list[threading.Lock] = []
        self._threads: list[threading.Thread] = []
        self._running = False

        self._state_lock = threading.Lock()
        self._states: dict[int, VehicleState] = {}
        self._sysid_conn: dict[int, int] = {}
        self._requested: set[int] = set()

    # ------------------------------------------------------------------ #
    async def start(self) -> None:
        self._running = True
        for cs in self.conn_strings:
            conn = self.mavutil.mavlink_connection(cs, source_system=self.source_system,
                                                   autoreconnect=True)
            self._conns.append(conn)
            self._locks.append(threading.Lock())
        for idx in range(len(self._conns)):
            t = threading.Thread(target=self._reader, args=(idx,), daemon=True)
            t.start()
            self._threads.append(t)

    async def stop(self) -> None:
        self._running = False
        for conn in self._conns:
            try:
                conn.close()
            except Exception:
                pass

    # ------------------------------------------------------------------ #
    def _reader(self, idx: int) -> None:
        conn = self._conns[idx]
        while self._running:
            try:
                msg = conn.recv_match(blocking=True, timeout=1.0)
            except Exception:
                time.sleep(0.2)
                continue
            if msg is None:
                continue
            self._ingest(idx, conn, msg)

    def _ingest(self, idx: int, conn, msg) -> None:
        sysid = msg.get_srcSystem()
        if sysid == 0 or sysid == 255:
            return
        mtype = msg.get_type()
        now = time.monotonic()
        with self._state_lock:
            st = self._states.get(sysid)
            if st is None:
                st = VehicleState(sysid=sysid, name=self.names.get(sysid, f"D{sysid}"),
                                  profile_id=self.sysid_profiles.get(sysid, "default"))
                self._states[sysid] = st
                self._sysid_conn[sysid] = idx

            if mtype == "HEARTBEAT":
                st.last_heartbeat = now
                st.autopilot = _AP_MAP.get(msg.autopilot, Autopilot.UNKNOWN)
                st.frame = _FRAME_MAP.get(msg.type, FrameType.UNKNOWN)
                st.armed = bool(msg.base_mode & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                st.mode = self._mode_name(conn, msg.custom_mode)
            elif mtype == "GLOBAL_POSITION_INT":
                st.lat = msg.lat / 1e7
                st.lon = msg.lon / 1e7
                st.alt_msl = msg.alt / 1000.0
                st.alt_rel = msg.relative_alt / 1000.0
                st.vx = msg.vy / 100.0          # MAVLink vy = east  -> ENU x
                st.vy = msg.vx / 100.0          # MAVLink vx = north -> ENU y
                st.vz = -msg.vz / 100.0         # MAVLink vz = down  -> up
                st.heading = (msg.hdg / 100.0) if msg.hdg != 65535 else st.heading
                st.last_position = now
            elif mtype == "GPS_RAW_INT":
                st.fix_type = msg.fix_type
                st.num_sats = msg.satellites_visible
            elif mtype == "SYS_STATUS":
                st.battery_v = msg.voltage_battery / 1000.0
                st.battery_pct = msg.battery_remaining if msg.battery_remaining != -1 else -1.0
            elif mtype == "AUTOPILOT_VERSION":
                st.capabilities = msg.capabilities
                fr = msg.flight_sw_version
                st.fw_version = f"{(fr>>24)&0xFF}.{(fr>>16)&0xFF}.{(fr>>8)&0xFF}"

        # first time we hear a vehicle: ask for streams + its version
        if sysid not in self._requested:
            self._requested.add(sysid)
            self._request_streams(idx, conn, sysid)

    def _mode_name(self, conn, custom_mode: int) -> str:
        try:
            inv = {v: k for k, v in conn.mode_mapping().items()}
            return inv.get(custom_mode, str(custom_mode))
        except Exception:
            return str(custom_mode)

    def _request_streams(self, idx: int, conn, sysid: int) -> None:
        with self._locks[idx]:
            try:
                conn.mav.request_data_stream_send(
                    sysid, 1, self.mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    self.stream_rate_hz, 1)
                conn.mav.command_long_send(
                    sysid, 1, self.mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                    0, 1, 0, 0, 0, 0, 0, 0)
            except Exception:
                pass

    # ------------------------------------------------------------------ #
    def snapshot(self) -> dict[int, VehicleState]:
        with self._state_lock:
            return {sid: st.copy() for sid, st in self._states.items()}

    def _conn_for(self, sysid: int):
        idx = self._sysid_conn.get(sysid, 0)
        return idx, self._conns[idx]

    async def send_setpoint(self, sysid: int, target: WorldTarget) -> None:
        idx, conn = self._conn_for(sysid)
        m = self.mavutil.mavlink
        # position-only mask, plus yaw if provided
        type_mask = 0b0000_1111_1111_1000  # ignore vel/acc
        if target.yaw_deg is None:
            type_mask |= (1 << 10) | (1 << 11)
        yaw = (target.yaw_deg or 0.0) * 3.14159265 / 180.0
        with self._locks[idx]:
            try:
                conn.mav.set_position_target_global_int_send(
                    0, sysid, 1, m.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, type_mask,
                    int(target.lat * 1e7), int(target.lon * 1e7), float(target.alt_rel),
                    0, 0, 0, 0, 0, 0, yaw, 0)
            except Exception:
                pass

    async def set_mode(self, sysid: int, mode: str) -> None:
        idx, conn = self._conn_for(sysid)
        try:
            mapping = conn.mode_mapping()
            if mode in mapping:
                with self._locks[idx]:
                    conn.mav.set_mode_send(
                        sysid, self.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mapping[mode])
        except Exception:
            pass

    async def arm(self, sysid: int, arm: bool = True) -> None:
        idx, conn = self._conn_for(sysid)
        m = self.mavutil.mavlink
        with self._locks[idx]:
            try:
                conn.mav.command_long_send(sysid, 1, m.MAV_CMD_COMPONENT_ARM_DISARM,
                                           0, 1 if arm else 0, 0, 0, 0, 0, 0, 0)
            except Exception:
                pass

    async def takeoff(self, sysid: int, alt_m: float) -> None:
        await self.set_mode(sysid, "GUIDED")
        await self.arm(sysid, True)
        idx, conn = self._conn_for(sysid)
        m = self.mavutil.mavlink
        with self._locks[idx]:
            try:
                conn.mav.command_long_send(sysid, 1, m.MAV_CMD_NAV_TAKEOFF,
                                           0, 0, 0, 0, 0, 0, 0, alt_m)
            except Exception:
                pass

    async def land(self, sysid: int) -> None:
        await self.set_mode(sysid, "LAND")
