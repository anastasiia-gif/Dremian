#!/usr/bin/env python3
"""
=============================================================
  DUAL-DRONE FORMATION CONTROLLER  v3
=============================================================
  - Sets all ArduPilot params itself (no MAVProxy needed)
  - Works with old ArduCopter 3.3 (dronekit-sitl) AND modern fw
  - GPS sysid filter removed (one port = one drone, no ambiguity)
  - PS4 / PS5 / TX16S joystick profiles built in

USAGE (SITL):
  python formation_controller_v3.py --joystick ps5 --arm

USAGE (real hardware):
  python formation_controller_v3.py \
    --port1 COM3 --port2 COM4 --joystick tx16s --arm

STICK MAPPING (PS4/PS5):
  Left  stick X  → rotate heading
  Left  stick Y  → altitude up/down
  Right stick X  → strafe left/right
  Right stick Y  → forward/backward
  R2 trigger     → separation distance
=============================================================
"""

import math, time, threading, argparse, sys
from dataclasses import dataclass

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pip install pymavlink"); sys.exit(1)
try:
    import pygame
except ImportError:
    print("ERROR: pip install pygame"); sys.exit(1)

# ─── Formation defaults ────────────────────────────────────────────────────
DEFAULT_SEP_M     = 3.0
MIN_SEP_M         = 1.5
MAX_SEP_M         = 10.0
DEFAULT_ALT_M     = 3.0
MIN_ALT_M         = 2.0
MAX_ALT_M         = 50.0
MAX_SPEED_H_MS    = 3.0
MAX_SPEED_V_MS    = 1.0
MAX_YAW_RATE_DPS  = 30.0
UPDATE_HZ         = 10
JOYSTICK_DEADBAND = 0.06
DRONE1_SYSID      = 1
DRONE2_SYSID      = 2

# ─── Joystick profiles ────────────────────────────────────────────────────
@dataclass
class JoyProfile:
    name:       str
    axis_roll:  int
    axis_pitch: int
    axis_thr:   int
    axis_yaw:   int
    axis_sep:   int
    inv_pitch:  bool = True
    inv_thr:    bool = True

PROFILES = {
    "tx16s": JoyProfile("TX16S",          axis_roll=0, axis_pitch=1,
                        axis_thr=2, axis_yaw=3, axis_sep=4),
    "ps4":   JoyProfile("PS4 DualShock4", axis_roll=2, axis_pitch=3,
                        axis_thr=1, axis_yaw=0, axis_sep=5),
    "ps5":   JoyProfile("PS5 DualSense",  axis_roll=2, axis_pitch=3,
                        axis_thr=1, axis_yaw=0, axis_sep=5),
}

# ─── Helpers ──────────────────────────────────────────────────────────────
def deadband(v, db=JOYSTICK_DEADBAND):
    return v if abs(v) > db else 0.0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def offset_gps(lat, lon, bearing_deg, dist_m):
    R = 6371000.0
    b = math.radians(bearing_deg)
    lat1, lon1 = math.radians(lat), math.radians(lon)
    d_R = dist_m / R
    lat2 = math.asin(math.sin(lat1)*math.cos(d_R) +
                     math.cos(lat1)*math.sin(d_R)*math.cos(b))
    lon2 = lon1 + math.atan2(math.sin(b)*math.sin(d_R)*math.cos(lat1),
                              math.cos(d_R)-math.sin(lat1)*math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dlat = math.radians(lat2-lat1); dlon = math.radians(lon2-lon1)
    a = (math.sin(dlat/2)**2 +
         math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2)**2)
    return R*2*math.atan2(math.sqrt(a), math.sqrt(1-a))

# ─── Drone connection ─────────────────────────────────────────────────────
class DroneLink:
    def __init__(self, conn_str, baud, sysid, name):
        self.name     = name
        self.sysid    = sysid
        self.conn_str = conn_str
        self.baud     = baud
        self.conn     = None
        self.lat      = None
        self.lon      = None
        self.alt      = None
        self._lock    = threading.Lock()

    def connect(self, timeout=30):
        is_serial = not any(p in self.conn_str for p in ['udp','tcp',':/'])
        print(f"[{self.name}] Connecting: {self.conn_str} ...")
        try:
            kw = dict(baud=self.baud) if is_serial else {}
            self.conn = mavutil.mavlink_connection(
                self.conn_str, source_system=255, source_component=190, **kw)
            hb = self.conn.wait_heartbeat(timeout=timeout)
            if hb is None:
                print(f"[{self.name}] No heartbeat in {timeout}s")
                return False
            # Use whatever sysid the vehicle actually reports
            actual_sysid = hb.get_srcSystem()
            self.conn.target_system    = actual_sysid
            self.conn.target_component = 1
            self.sysid = actual_sysid
            print(f"[{self.name}] Connected ✓  (sysid={actual_sysid})")
            return True
        except Exception as e:
            print(f"[{self.name}] Error: {e}")
            return False

    def setup_params(self):
        """
        Set all required params via MAVLink param_set.
        Uses REQUEST_DATA_STREAM (works on old AND new firmware).
        """
        print(f"[{self.name}] Setting params ...")
        params = {
            'ARMING_CHECK': 0,
            'FENCE_ENABLE': 0,
            'FS_GCS_ENABLE': 0,
            'FS_THR_ENABLE': 0,
            'WP_SPEED': 300,
            'WPNAV_ACCEL': 150,
        }
        for pname, value in params.items():
            self.conn.mav.param_set_send(
                self.sysid, 1,
                pname.encode('utf-8'),
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            time.sleep(0.15)

        # Force GPS + position data stream (works on ArduCopter 3.3+)
        self.conn.mav.request_data_stream_send(
            self.sysid, 1,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,   # 10 Hz
            1     # start
        )
        print(f"[{self.name}] Params set, streaming at 10 Hz")

    def get_gps(self):
        """Read GPS — no sysid filter, one port = one drone."""
        msg = self.conn.recv_match(type='GLOBAL_POSITION_INT',
                                   blocking=True, timeout=1.0)
        if msg:
            with self._lock:
                self.lat = msg.lat / 1e7
                self.lon = msg.lon / 1e7
                self.alt = msg.relative_alt / 1000.0
            return True
        return False

    def set_mode(self, mode):
        mm = self.conn.mode_mapping()
        if mode not in mm:
            print(f"[{self.name}] Unknown mode '{mode}'")
            return
        self.conn.mav.set_mode_send(
            self.sysid,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mm[mode]
        )
        print(f"[{self.name}] Mode → {mode}")
        time.sleep(0.5)

    def arm(self, state=True):
        self.conn.mav.command_long_send(
            self.sysid, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if state else 0,
            0, 0, 0, 0, 0, 0
        )
        print(f"[{self.name}] {'ARMED' if state else 'DISARMED'}")

    def takeoff(self, alt):
        self.conn.mav.command_long_send(
            self.sysid, 1,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt
        )
        print(f"[{self.name}] Takeoff → {alt}m")

    def land(self):
        self.set_mode('RTL')

    def send_position(self, lat, lon, alt):
        self.conn.mav.set_position_target_global_int_send(
            0, self.sysid, 1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0x0FF8,
            int(lat*1e7), int(lon*1e7), alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

# ─── Formation controller ─────────────────────────────────────────────────
class FormationController:
    def __init__(self, d1, d2, sep, profile: JoyProfile):
        self.d1         = d1
        self.d2         = d2
        self.separation = sep
        self.profile    = profile
        self.mid_lat    = None
        self.mid_lon    = None
        self.mid_alt    = DEFAULT_ALT_M
        self.heading    = 0.0
        self.joy        = None
        self._running   = False

    def setup_joystick(self):
        pygame.init()
        pygame.joystick.init()
        n = pygame.joystick.get_count()
        if n == 0:
            print("ERROR: No joystick found.")
            return False
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        print(f"Joystick: '{self.joy.get_name()}'  profile: {self.profile.name}")
        print(f"  Axes: {self.joy.get_numaxes()}  Buttons: {self.joy.get_numbuttons()}")
        return True

    def init_midpoint(self):
        print("Waiting for GPS from both drones ...")
        timeout = 30
        t0 = time.time()
        while self.d1.lat is None or self.d2.lat is None:
            self.d1.get_gps()
            self.d2.get_gps()
            if time.time() - t0 > timeout:
                print("GPS timeout! Check stream rate. Continuing with defaults.")
                if self.d1.lat is None: self.d1.lat = 52.370216; self.d1.lon = 4.895168; self.d1.alt = 0
                if self.d2.lat is None: self.d2.lat = 52.370216; self.d2.lon = 4.898168; self.d2.alt = 0
                break
            time.sleep(0.2)
        self.mid_lat = (self.d1.lat + self.d2.lat) / 2
        self.mid_lon = (self.d1.lon + self.d2.lon) / 2
        self.mid_alt = DEFAULT_ALT_M
        print(f"Midpoint: {self.mid_lat:.6f}, {self.mid_lon:.6f}  alt={self.mid_alt:.1f}m")

    def _telem_loop(self):
        while self._running:
            try:
                self.d1.get_gps()
                self.d2.get_gps()
            except Exception:
                pass
            time.sleep(0.05)

    def read_joy(self):
        pygame.event.pump()
        p = self.profile
        n = self.joy.get_numaxes()
        def ax(i): return deadband(self.joy.get_axis(i)) if i < n else 0.0
        roll  = ax(p.axis_roll)
        pitch = (-1 if p.inv_pitch else 1) * ax(p.axis_pitch)
        thr   = (-1 if p.inv_thr   else 1) * ax(p.axis_thr)
        yaw   = ax(p.axis_yaw)
        sep_t = (ax(p.axis_sep) + 1) / 2   # -1..+1 → 0..1
        return dict(roll=roll, pitch=pitch, throttle=thr, yaw=yaw, sep_t=sep_t)

    def compute_positions(self):
        right = (self.heading + 90) % 360
        left  = (self.heading - 90) % 360
        h     = self.separation / 2
        lat1, lon1 = offset_gps(self.mid_lat, self.mid_lon, right, h)
        lat2, lon2 = offset_gps(self.mid_lat, self.mid_lon, left,  h)
        return (lat1, lon1, self.mid_alt), (lat2, lon2, self.mid_alt)

    def run(self):
        dt = 1.0 / UPDATE_HZ
        n  = 0
        self._running = True
        telem = threading.Thread(target=self._telem_loop, daemon=True)
        telem.start()
        print("\n" + "="*58)
        print("  FORMATION RUNNING  |  Ctrl+C = emergency RTL")
        print("="*58 + "\n")
        try:
            while self._running:
                t0 = time.time()

                joy = self.read_joy()
                self.separation = clamp(
                    MIN_SEP_M + joy['sep_t'] * (MAX_SEP_M - MIN_SEP_M),
                    MIN_SEP_M, MAX_SEP_M)
                self.heading = (self.heading + joy['yaw'] * MAX_YAW_RATE_DPS * dt) % 360

                hr  = math.radians(self.heading)
                vn  = (joy['pitch']*math.cos(hr) - joy['roll']*math.sin(hr)) * MAX_SPEED_H_MS
                ve  = (joy['pitch']*math.sin(hr) + joy['roll']*math.cos(hr)) * MAX_SPEED_H_MS
                vu  = joy['throttle'] * MAX_SPEED_V_MS

                lat_r = math.radians(self.mid_lat)
                self.mid_lat += vn / 111111.0 * dt
                self.mid_lon += ve / (111111.0 * math.cos(lat_r)) * dt
                self.mid_alt  = clamp(self.mid_alt + vu * dt, MIN_ALT_M, MAX_ALT_M)

                p1, p2 = self.compute_positions()
                self.d1.send_position(*p1)
                self.d2.send_position(*p2)

                n += 1
                if n % UPDATE_HZ == 0:
                    dist = (haversine_m(self.d1.lat or self.mid_lat,
                                        self.d1.lon or self.mid_lon,
                                        self.d2.lat or self.mid_lat,
                                        self.d2.lon or self.mid_lon)
                            if self.d1.lat and self.d2.lat else 0.0)
                    print(f"  Mid {self.mid_lat:.5f},{self.mid_lon:.5f}  "
                          f"Alt={self.mid_alt:.1f}m  Hdg={self.heading:.0f}deg  "
                          f"Sep_cmd={self.separation:.1f}m  Act={dist:.1f}m")

                s = dt - (time.time() - t0)
                if s > 0:
                    time.sleep(s)

        except KeyboardInterrupt:
            print("\n[CTRL+C] RTL both drones ...")
        finally:
            self._running = False
            self.d1.land()
            self.d2.land()
            pygame.quit()

# ─── Axis mapper ──────────────────────────────────────────────────────────
def run_axis_mapper():
    pygame.init(); pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick found."); return
    j = pygame.joystick.Joystick(0); j.init()
    print(f"Controller: {j.get_name()}")
    print(f"Axes: {j.get_numaxes()}  Buttons: {j.get_numbuttons()}\n")
    print("Move sticks slowly. Ctrl+C to exit.\n")
    print("  " + "  ".join(f"Ax{i:02d}" for i in range(j.get_numaxes())))
    try:
        while True:
            pygame.event.pump()
            axes  = [j.get_axis(i) for i in range(j.get_numaxes())]
            parts = [f"[{v:+.2f}]" if abs(v) > 0.1 else f" {v:+.2f} " for v in axes]
            print(f"\r  " + " ".join(parts), end='', flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nDone. Update PROFILES in the script if your axes differ.")
        pygame.quit()

# ─── CLI ──────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description="Dual-drone formation controller",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--port1',     default='udp:127.0.0.1:14551')
    p.add_argument('--port2',     default='udp:127.0.0.1:14561')
    p.add_argument('--baud',      type=int,   default=57600)
    p.add_argument('--sep',       type=float, default=DEFAULT_SEP_M)
    p.add_argument('--alt',       type=float, default=DEFAULT_ALT_M)
    p.add_argument('--joystick',  default='ps5',
                   choices=['tx16s','ps4','ps5','auto'])
    p.add_argument('--arm',       action='store_true',
                   help='Auto-arm and takeoff on launch')
    p.add_argument('--sim',       action='store_true',
                   help='Geometry-only sim, no drones needed')
    return p.parse_args()

def main():
    args = parse_args()

    if args.joystick == 'auto':
        run_axis_mapper(); return

    profile = PROFILES[args.joystick]
    print(f"Joystick profile: {profile.name}")

    # ── Geometry sim mode ─────────────────────────────────────────────────
    if args.sim:
        print("[SIM MODE] No drones needed. Ctrl+C to stop.\n")
        pygame.init(); pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("No joystick found."); return
        j = pygame.joystick.Joystick(0); j.init()
        mid_lat, mid_lon, mid_alt = 52.370216, 4.895168, 8.0
        heading, separation = 0.0, args.sep
        dt = 1.0 / UPDATE_HZ
        try:
            while True:
                pygame.event.pump()
                n = j.get_numaxes()
                def ax(i): return deadband(j.get_axis(i)) if i < n else 0.0
                roll  = ax(profile.axis_roll)
                pitch = (-1 if profile.inv_pitch else 1) * ax(profile.axis_pitch)
                thr   = (-1 if profile.inv_thr   else 1) * ax(profile.axis_thr)
                yaw   = ax(profile.axis_yaw)
                sep_t = (ax(profile.axis_sep) + 1) / 2
                separation = clamp(MIN_SEP_M + sep_t*(MAX_SEP_M-MIN_SEP_M), MIN_SEP_M, MAX_SEP_M)
                heading = (heading + yaw * MAX_YAW_RATE_DPS * dt) % 360
                hr = math.radians(heading)
                vn = (pitch*math.cos(hr) - roll*math.sin(hr)) * MAX_SPEED_H_MS
                ve = (pitch*math.sin(hr) + roll*math.cos(hr)) * MAX_SPEED_H_MS
                mid_lat += vn / 111111.0 * dt
                mid_lon += ve / (111111.0 * math.cos(math.radians(mid_lat))) * dt
                mid_alt  = clamp(mid_alt + thr * MAX_SPEED_V_MS * dt, MIN_ALT_M, MAX_ALT_M)
                r = (heading+90)%360; l = (heading-90)%360; h = separation/2
                lat1, lon1 = offset_gps(mid_lat, mid_lon, r, h)
                lat2, lon2 = offset_gps(mid_lat, mid_lon, l, h)
                print(f"\r Hdg={heading:5.1f}deg Alt={mid_alt:4.1f}m Sep={separation:.1f}m "
                      f"D1=({lat1:.5f},{lon1:.5f}) D2=({lat2:.5f},{lon2:.5f})", end='')
                time.sleep(dt)
        except KeyboardInterrupt:
            print("\nSim ended."); pygame.quit(); return

    # ── Real / SITL mode ──────────────────────────────────────────────────
    d1 = DroneLink(args.port1, args.baud, DRONE1_SYSID, "D1")
    d2 = DroneLink(args.port2, args.baud, DRONE2_SYSID, "D2")

    if not d1.connect(): sys.exit(1)
    if not d2.connect(): sys.exit(1)

    # Script sets all params itself — no MAVProxy console commands needed
    d1.setup_params()
    d2.setup_params()
    time.sleep(2)

    fc = FormationController(d1, d2, args.sep, profile)
    fc.mid_alt = args.alt

    if not fc.setup_joystick(): sys.exit(1)

    fc.init_midpoint()

    if args.arm:
        print("\nWARNING: Auto-arm in 5s ... Ctrl+C to abort")
        for i in range(5, 0, -1):
            print(f"  {i}..."); time.sleep(1)
        d1.set_mode('GUIDED')
        d2.set_mode('GUIDED')
        time.sleep(1)
        d1.arm()
        d2.arm()
        time.sleep(2)
        d1.takeoff(args.alt)
        d2.takeoff(args.alt)
        print(f"Climbing to {args.alt}m ..."); time.sleep(8)

    fc.run()

if __name__ == '__main__':
    main()