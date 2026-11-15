#!/usr/bin/env python3
"""
drone_sim.py  —  Two virtual MAVLink drones over TCP
Drone 1 → TCP port 5760
Drone 2 → TCP port 5770

Terminal 1:  python drone_sim.py
Terminal 2:  python formation_controller_v3.py --joystick ps5 --arm
             --port1 tcp:127.0.0.1:5760 --port2 tcp:127.0.0.1:5770
"""

import math, time, threading
from pymavlink import mavutil

DRONES = [
    {"name": "D1", "sysid": 1, "port": 5760,
     "lat": -35.363261, "lon": 149.165230, "alt": 0.0},
    {"name": "D2", "sysid": 2, "port": 5770,
     "lat": -35.363461, "lon": 149.165230, "alt": 0.0},
]

class VirtualDrone:
    def __init__(self, cfg):
        self.name      = cfg["name"]
        self.sysid     = cfg["sysid"]
        self.port      = cfg["port"]
        self.lat       = cfg["lat"]
        self.lon       = cfg["lon"]
        self.alt       = cfg["alt"]
        self.armed     = False
        self.mode      = 0
        self.mode_name = "STABILIZE"
        self._lock     = threading.Lock()
        self.conn      = None

    def _wait_for_connection(self):
        print(f"[{self.name}] Waiting for connection on TCP port {self.port} ...")
        self.conn = mavutil.mavlink_connection(
            f"tcpin:0.0.0.0:{self.port}",
            source_system=self.sysid,
            source_component=1
        )
        # Don't wait for heartbeat — just accept TCP and start sending immediately
        # The script will receive our heartbeat and connect
        print(f"[{self.name}] TCP ready, sending heartbeats...")

    def _send_heartbeat(self):
        self.conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            0, self.mode,
            mavutil.mavlink.MAV_STATE_ACTIVE if self.armed
            else mavutil.mavlink.MAV_STATE_STANDBY
        )

    def _send_gps(self):
        with self._lock:
            lat, lon, alt = self.lat, self.lon, self.alt
        ts = int(time.time() * 1000) & 0xFFFFFFFF
        self.conn.mav.global_position_int_send(
            ts,
            int(lat * 1e7), int(lon * 1e7),
            int(alt * 1000), int(alt * 1000),
            0, 0, 0, 65535
        )
        self.conn.mav.gps_raw_int_send(
            ts, 3,
            int(lat * 1e7), int(lon * 1e7),
            int(alt * 1000),
            100, 100, 0, 0, 10
        )

    def _handle_messages(self):
        while True:
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.1)
                if msg is None:
                    continue
                t = msg.get_type()

                if t == "COMMAND_LONG":
                    cmd = msg.command
                    if cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        self.armed = (msg.param1 == 1)
                        print(f"[{self.name}] {'ARMED' if self.armed else 'DISARMED'}")
                        self.conn.mav.command_ack_send(cmd, 0)
                    elif cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                        with self._lock:
                            self.alt = msg.param7
                        print(f"[{self.name}] TAKEOFF to {msg.param7}m")
                        self.conn.mav.command_ack_send(cmd, 0)
                    else:
                        self.conn.mav.command_ack_send(cmd, 0)

                elif t == "SET_POSITION_TARGET_GLOBAL_INT":
                    with self._lock:
                        self.lat = msg.lat_int / 1e7
                        self.lon = msg.lon_int / 1e7
                        self.alt = msg.alt

                elif t == "SET_MODE":
                    self.mode = msg.custom_mode
                    names = {0:"STABILIZE", 3:"AUTO", 4:"GUIDED",
                             5:"LOITER", 6:"RTL"}
                    self.mode_name = names.get(self.mode, str(self.mode))
                    print(f"[{self.name}] Mode set to {self.mode_name}")

                elif t == "PARAM_SET":
                    self.conn.mav.param_value_send(
                        msg.param_id, msg.param_value,
                        msg.param_type, 1, 0)

            except Exception:
                pass

    def _status_loop(self):
        while True:
            with self._lock:
                lat, lon, alt = self.lat, self.lon, self.alt
            print(f"  [{self.name}] lat={lat:.6f}  lon={lon:.6f}  "
                  f"alt={alt:.1f}m  armed={self.armed}  mode={self.mode_name}")
            time.sleep(1)

    def run(self):
        self._wait_for_connection()
        threading.Thread(target=self._handle_messages, daemon=True).start()
        threading.Thread(target=self._status_loop,    daemon=True).start()
        last_hb = 0
        while True:
            now = time.time()
            if now - last_hb >= 1.0:
                self._send_heartbeat()
                last_hb = now
            self._send_gps()
            time.sleep(0.1)

def main():
    print("=" * 50)
    print("  VIRTUAL DRONE SIMULATOR")
    print("  D1 -> TCP 5760")
    print("  D2 -> TCP 5770")
    print("  Starting both, waiting for script to connect...")
    print("=" * 50 + "\n")

    drones = [VirtualDrone(cfg) for cfg in DRONES]

    t = threading.Thread(target=drones[1].run, daemon=False)
    t.start()
    drones[0].run()

if __name__ == "__main__":
    main()