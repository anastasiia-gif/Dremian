"""
A fake ArduCopter-flavoured MAVLink endpoint, faithful to the bits that matter for a GCS.

It speaks genuine MAVLink over UDP (HEARTBEAT / GLOBAL_POSITION_INT / GPS_RAW_INT /
SYS_STATUS / AUTOPILOT_VERSION) and, crucially, it models ArduCopter's GUIDED-mode
launch rules so tests behave like the real thing:

  * It will NOT climb in response to position setpoints alone. To leave the ground it must
    receive MAV_CMD_NAV_TAKEOFF while armed and in GUIDED (mode 4).
  * If it is armed but no takeoff starts within ~15 s, the motors auto-disarm -- exactly
    the "arms, waits, disarms" behaviour real Copters show when you forget the takeoff.
  * In LAND mode (9) it descends and disarms on the ground.

Dynamics are otherwise a crude first-order chase. For real flight dynamics use SITL.
"""
from __future__ import annotations

import math
import threading
import time

from pymavlink import mavutil

GUIDED = 4
LAND = 9
ARM_GRACE_S = 12.0          # ArduCopter auto-disarms ~15 s after arming if not flying


class FakeCopter:
    def __init__(self, sysid, connstr, lat, lon, alt_msl=488.0, alt_rel=0.0,
                 move_gain=0.25):
        self.sysid = sysid
        self.m = mavutil.mavlink_connection(connstr, source_system=sysid, source_component=1)
        self.lat, self.lon = lat, lon
        self.alt_msl = alt_msl
        self.alt_rel = alt_rel
        self.hdg = 0.0
        self.armed = False
        self.custom_mode = 0
        self.tgt = None               # last position setpoint (lat, lon, alt_rel)
        self.move_gain = move_gain
        # flight state machine
        self.airborne = False
        self.taking_off = False
        self.takeoff_alt = 0.0
        self.arm_time = None
        # observability for tests
        self.setpoints = 0
        self.got_arm_cmd = False
        self.got_takeoff = False
        self.auto_disarmed = False
        self.running = True
        self.boot = time.time()

    def start(self):
        threading.Thread(target=self._tx, daemon=True).start()
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._move, daemon=True).start()
        return self

    def stop(self):
        self.running = False
        try:
            self.m.close()
        except Exception:
            pass

    # ---- telemetry out ----
    def _tx(self):
        mav, ml = self.m.mav, mavutil.mavlink
        while self.running:
            base = ml.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            if self.armed:
                base |= ml.MAV_MODE_FLAG_SAFETY_ARMED
            mav.heartbeat_send(ml.MAV_TYPE_QUADROTOR, ml.MAV_AUTOPILOT_ARDUPILOTMEGA,
                               base, self.custom_mode, ml.MAV_STATE_ACTIVE)
            t_ms = int((time.time() - self.boot) * 1000)
            mav.global_position_int_send(t_ms, int(self.lat * 1e7), int(self.lon * 1e7),
                                         int((self.alt_msl + self.alt_rel) * 1000),
                                         int(self.alt_rel * 1000),
                                         0, 0, 0, int(self.hdg * 100) % 36000)
            mav.gps_raw_int_send(t_ms * 1000, 3, int(self.lat * 1e7), int(self.lon * 1e7),
                                 int((self.alt_msl + self.alt_rel) * 1000),
                                 65535, 65535, 0, 0, 12)
            mav.sys_status_send(0, 0, 0, 250, 12000, -1, 88, 0, 0, 0, 0, 0, 0)
            time.sleep(0.1)

    # ---- command in ----
    def _rx(self):
        ml = mavutil.mavlink
        while self.running:
            try:
                msg = self.m.recv_match(blocking=True, timeout=0.4)
            except Exception:
                continue
            if msg is None:
                continue
            t = msg.get_type()
            if t == "SET_MODE" and msg.target_system == self.sysid:
                self.custom_mode = msg.custom_mode
            elif t == "COMMAND_LONG" and msg.target_system in (0, self.sysid):
                if msg.command == ml.MAV_CMD_COMPONENT_ARM_DISARM:
                    self._set_armed(int(msg.param1) == 1)
                    self.got_arm_cmd = True
                    self._ack(msg.command)
                elif msg.command == ml.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    self._send_version()
                    self._ack(msg.command)
                elif msg.command == ml.MAV_CMD_NAV_TAKEOFF:
                    # only a real GUIDED takeoff lifts the vehicle
                    if self.armed and self.custom_mode == GUIDED:
                        self.taking_off = True
                        self.takeoff_alt = msg.param7 or 2.0
                        self.got_takeoff = True
                    self._ack(msg.command)
            elif t == "SET_POSITION_TARGET_GLOBAL_INT" and msg.target_system == self.sysid:
                self.setpoints += 1
                self.tgt = (msg.lat_int / 1e7, msg.lon_int / 1e7, float(msg.alt))
                if msg.yaw:
                    self.hdg = math.degrees(msg.yaw) % 360.0

    def _set_armed(self, value):
        self.armed = value
        if value:
            self.arm_time = time.time()
        else:
            self.airborne = self.taking_off = False

    def _ack(self, cmd):
        try:
            self.m.mav.command_ack_send(cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED)
        except Exception:
            pass

    def _send_version(self):
        caps = mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT
        try:
            self.m.mav.autopilot_version_send(caps, (4 << 24) | (5 << 16), 0, 0, 0,
                                              [0] * 8, [0] * 8, [0] * 8, 0, 0, 0)
        except Exception:
            pass

    # ---- flight model ----
    def _move(self):
        while self.running:
            now = time.time()
            # auto-disarm: armed, never took off, grace elapsed
            if (self.armed and not self.airborne and not self.taking_off
                    and self.arm_time and now - self.arm_time > ARM_GRACE_S):
                self._set_armed(False)
                self.auto_disarmed = True

            if self.armed:
                k = self.move_gain
                if self.taking_off:
                    self.alt_rel += (self.takeoff_alt - self.alt_rel) * k
                    if self.alt_rel >= self.takeoff_alt - 0.1:
                        self.airborne = True
                        self.taking_off = False
                        self.tgt = None
                elif self.custom_mode == LAND and self.airborne:
                    self.alt_rel += (0.0 - self.alt_rel) * k
                    if self.alt_rel < 0.2:
                        self._set_armed(False)        # land -> disarm on the ground
                elif self.airborne and self.tgt:
                    tl, tn, ta = self.tgt
                    self.lat += (tl - self.lat) * k
                    self.lon += (tn - self.lon) * k
                    self.alt_rel += (ta - self.alt_rel) * k
            time.sleep(0.05)
