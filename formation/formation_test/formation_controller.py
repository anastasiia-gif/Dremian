#!/usr/bin/env python3
"""
=============================================================
  DUAL-DRONE FORMATION CONTROLLER
  ArduPilot + RadioMaster TX16S + pymavlink + pygame
  
  Formation: Two drones held as a rigid "stick" in the air,
  side-by-side (perpendicular to flight heading).
  The RC controller moves the virtual midpoint.
  Cloth/net can be strung between the two drones.
=============================================================

AXIS MAPPING (RadioMaster TX16S, Mode 2, USB Joystick):
  Axis 0  = Roll       (Right stick, X)  → Strafe L/R
  Axis 1  = Pitch      (Right stick, Y)  → Forward/Back  [inverted]
  Axis 2  = Throttle   (Left stick, Y)   → Altitude up/down [inverted]
  Axis 3  = Yaw        (Left stick, X)   → Rotate formation heading
  Axis 4+ = Aux/Knobs                    → Separation distance

CONTROLS:
  Right Stick  = Move midpoint (forward/back/strafe)
  Left  Stick  = Altitude (up/down) + Rotate heading (L/R)
  SA switch    = ARM all drones (3-pos switch, axis 4 or button)
  SE switch    = DISARM (safety)
  Knob (VRA)   = Separation distance (1.5 m → 10 m)

SAFETY:
  Press Ctrl+C or flip SE switch to disarm and land.
"""

import math
import time
import threading
import argparse
import sys
import os

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not installed. Run: pip install pymavlink")
    sys.exit(1)

try:
    import pygame
except ImportError:
    print("ERROR: pygame not installed. Run: pip install pygame")
    sys.exit(1)

# ─────────────────────────── CONFIGURATION ────────────────────────────────
# Serial ports for the two SiK telemetry radios (change to match your system)
# Windows example: 'COM3', 'COM4'
# Linux  example: '/dev/ttyUSB0', '/dev/ttyUSB1'
# macOS  example: '/dev/cu.usbserial-0001', '/dev/cu.usbserial-0002'
DRONE1_PORT   = 'COM3'
DRONE2_PORT   = 'COM4'
BAUD_RATE     = 57600          # Standard SiK baud rate

# ArduPilot SYSID_THISMAV for each drone (set these in Mission Planner first)
DRONE1_SYSID  = 1
DRONE2_SYSID  = 2

# Formation defaults
DEFAULT_SEP_M      = 3.0     # metres between drones
MIN_SEP_M          = 1.5     # minimum allowed separation
MAX_SEP_M          = 10.0    # maximum allowed separation
DEFAULT_ALT_M      = 8.0     # target takeoff/hover altitude (m AGL)
MIN_ALT_M          = 3.0     # altitude floor
MAX_ALT_M          = 50.0    # altitude ceiling

# Speed limits
MAX_SPEED_H_MS     = 3.0     # max horizontal speed m/s
MAX_SPEED_V_MS     = 1.0     # max vertical speed m/s
MAX_YAW_RATE_DPS   = 30.0    # max heading rotation deg/s

# Update rate for position commands sent to drones
UPDATE_HZ          = 10      # Hz  (10 = every 100 ms)
JOYSTICK_DEADBAND  = 0.06    # ignore stick noise below this value

# Axis indices — adjust if your stick mapping differs
AXIS_ROLL      = 0    # right stick X  → strafe
AXIS_PITCH     = 1    # right stick Y  → forward/back  (inverted)
AXIS_THROTTLE  = 2    # left  stick Y  → altitude      (inverted)
AXIS_YAW       = 3    # left  stick X  → heading rotate
AXIS_SEP_KNOB  = 4    # VRA knob      → separation

# ──────────────────────────── HELPERS ─────────────────────────────────────

def deadband(value: float, db: float = JOYSTICK_DEADBAND) -> float:
    """Zero out small joystick noise."""
    return value if abs(value) > db else 0.0

def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))

def offset_gps(lat: float, lon: float, bearing_deg: float, dist_m: float):
    """
    Vincenty-lite: offset a GPS coordinate by (bearing, distance).
    Returns (new_lat, new_lon) in decimal degrees.
    Accurate to ~1 cm for distances < 100 m.
    """
    R = 6371000.0  # Earth mean radius metres
    b = math.radians(bearing_deg)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    d_R = dist_m / R

    lat2 = math.asin(
        math.sin(lat1) * math.cos(d_R) +
        math.cos(lat1) * math.sin(d_R) * math.cos(b)
    )
    lon2 = lon1 + math.atan2(
        math.sin(b) * math.sin(d_R) * math.cos(lat1),
        math.cos(d_R) - math.sin(lat1) * math.sin(lat2)
    )
    return math.degrees(lat2), math.degrees(lon2)

def haversine_m(lat1, lon1, lat2, lon2) -> float:
    """Great-circle distance in metres."""
    R = 6371000.0
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat/2)**2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(d_lon/2)**2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# ───────────────────────── DRONE CONNECTION ───────────────────────────────

class DroneLink:
    """Wraps a single pymavlink connection to one ArduPilot drone."""

    def __init__(self, port: str, baud: int, sysid: int, name: str):
        self.name   = name
        self.sysid  = sysid
        self.port   = port
        self.conn   = None
        self.lat    = None
        self.lon    = None
        self.alt    = None
        self.armed  = False
        self.mode   = "UNKNOWN"
        self._lock  = threading.Lock()

    def connect(self, timeout: int = 30) -> bool:
        print(f"[{self.name}] Connecting on {self.port} @ {BAUD_RATE} baud …")
        try:
            self.conn = mavutil.mavlink_connection(
                self.port, baud=BAUD_RATE,
                source_system=255,    # GCS system ID
                source_component=190
            )
            hb = self.conn.wait_heartbeat(timeout=timeout)
            if hb is None:
                print(f"[{self.name}] ERROR: no heartbeat within {timeout}s")
                return False
            self.conn.target_system    = self.sysid
            self.conn.target_component = 1
            print(f"[{self.name}] Connected ✓  (sys={self.sysid})")
            return True
        except Exception as e:
            print(f"[{self.name}] Connection error: {e}")
            return False

    def get_gps(self) -> bool:
        """Fetch GPS position from drone, non-blocking. Returns True on fresh data."""
        msg = self.conn.recv_match(
            type='GLOBAL_POSITION_INT',
            blocking=True, timeout=1.0
        )
        if msg and msg.get_srcSystem() == self.sysid:
            with self._lock:
                self.lat = msg.lat / 1e7
                self.lon = msg.lon / 1e7
                self.alt = msg.relative_alt / 1000.0
            return True
        return False

    def set_mode(self, mode: str):
        """Switch flight mode (e.g. 'GUIDED', 'LOITER', 'LAND', 'RTL')."""
        mode_map = self.conn.mode_mapping()
        if mode not in mode_map:
            print(f"[{self.name}] Unknown mode '{mode}'")
            return
        mode_id = mode_map[mode]
        self.conn.mav.set_mode_send(
            self.sysid,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        self.mode = mode
        print(f"[{self.name}] Mode → {mode}")

    def arm(self, state: bool = True):
        """Arm (True) or disarm (False) the drone."""
        self.conn.mav.command_long_send(
            self.sysid, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if state else 0,  # param1: 1=arm, 0=disarm
            0, 0, 0, 0, 0, 0
        )
        self.armed = state
        print(f"[{self.name}] {'ARMED' if state else 'DISARMED'}")

    def takeoff(self, alt_m: float):
        """Command a takeoff to alt_m (requires GUIDED mode + armed)."""
        self.conn.mav.command_long_send(
            self.sysid, 1,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            alt_m
        )
        print(f"[{self.name}] Takeoff → {alt_m} m")

    def land(self):
        """Command RTL (Return to Launch) for safe landing."""
        self.set_mode('RTL')

    def send_position(self, lat: float, lon: float, alt: float):
        """
        Send SET_POSITION_TARGET_GLOBAL_INT — the core formation command.
        type_mask = 0b0000_1111_1111_1000 = 0x0FF8
        → only position fields used, velocity/accel/yaw ignored.
        Frame: MAV_FRAME_GLOBAL_RELATIVE_ALT_INT (6)
        """
        self.conn.mav.set_position_target_global_int_send(
            0,                  # time_boot_ms (ignored)
            self.sysid,         # target_system
            1,                  # target_component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0x0FF8,             # type_mask: position only
            int(lat * 1e7),     # lat_int  (1e7 degrees)
            int(lon * 1e7),     # lon_int  (1e7 degrees)
            alt,                # alt (metres above home)
            0, 0, 0,            # vx, vy, vz (ignored)
            0, 0, 0,            # afx, afy, afz (ignored)
            0,                  # yaw (ignored)
            0                   # yaw_rate (ignored)
        )

    def request_message_rate(self, msg_id: int, hz: float):
        """Ask the FC to stream a specific message at hz rate."""
        interval_us = int(1e6 / hz)
        self.conn.mav.command_long_send(
            self.sysid, 1,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(msg_id),
            float(interval_us),
            0, 0, 0, 0, 0
        )


# ─────────────────────── FORMATION CONTROLLER ─────────────────────────────

class FormationController:
    """
    Manages the virtual midpoint and sends position commands to both drones.
    The midpoint is driven by joystick input at UPDATE_HZ.
    """

    def __init__(self, drone1: DroneLink, drone2: DroneLink,
                 separation: float = DEFAULT_SEP_M):
        self.d1 = drone1
        self.d2 = drone2
        self.separation = separation

        # Virtual midpoint state (GPS)
        self.mid_lat  = None
        self.mid_lon  = None
        self.mid_alt  = DEFAULT_ALT_M
        self.heading  = 0.0    # degrees clockwise from North

        # Pygame joystick
        self.joy      = None
        self._running = False

        # Telemetry thread
        self._telem_thread = None

    # ── Setup ─────────────────────────────────────────────────────────────

    def setup_joystick(self) -> bool:
        pygame.init()
        pygame.joystick.init()
        n = pygame.joystick.get_count()
        if n == 0:
            print("ERROR: No joystick found. Connect RadioMaster via USB-C (top port)")
            print("       and select 'USB Joystick (HID)' on the TX screen.")
            return False
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        print(f"Joystick: '{self.joy.get_name()}'")
        print(f"  Axes: {self.joy.get_numaxes()}, "
              f"Buttons: {self.joy.get_numbuttons()}, "
              f"Hats: {self.joy.get_numhats()}")
        return True

    def init_midpoint(self):
        """
        Initialise the virtual midpoint from the average of both drone positions.
        Call after both drones have GPS lock.
        """
        print("Waiting for GPS from both drones …")
        while self.d1.lat is None or self.d2.lat is None:
            self.d1.get_gps()
            self.d2.get_gps()
            time.sleep(0.2)
        self.mid_lat = (self.d1.lat + self.d2.lat) / 2.0
        self.mid_lon = (self.d1.lon + self.d2.lon) / 2.0
        self.mid_alt = max(self.d1.alt or 0, self.d2.alt or 0) + DEFAULT_ALT_M
        print(f"Midpoint init: {self.mid_lat:.6f}, {self.mid_lon:.6f}, "
              f"alt={self.mid_alt:.1f} m")

    # ── Telemetry thread ──────────────────────────────────────────────────

    def _telem_loop(self):
        """Background thread: continuously refresh GPS from both drones."""
        while self._running:
            try:
                self.d1.get_gps()
                self.d2.get_gps()
            except Exception:
                pass
            time.sleep(0.05)

    # ── Joystick read ─────────────────────────────────────────────────────

    def read_joystick(self):
        """
        Returns dict of normalised stick values (-1 to +1) and booleans.
        Call pygame.event.pump() first.
        """
        pygame.event.pump()
        n = self.joy.get_numaxes()

        def axis(i):
            return deadband(self.joy.get_axis(i)) if i < n else 0.0

        roll      =  axis(AXIS_ROLL)
        pitch     = -axis(AXIS_PITCH)     # inverted
        throttle  = -axis(AXIS_THROTTLE)  # inverted
        yaw       =  axis(AXIS_YAW)

        # Separation knob: maps (-1 .. +1) → (MIN_SEP_M .. MAX_SEP_M)
        knob_raw  = axis(AXIS_SEP_KNOB)
        sep_t     = (knob_raw + 1.0) / 2.0   # 0..1
        separation = MIN_SEP_M + sep_t * (MAX_SEP_M - MIN_SEP_M)

        return {
            'roll':       roll,
            'pitch':      pitch,
            'throttle':   throttle,
            'yaw':        yaw,
            'separation': separation,
        }

    # ── Formation geometry ────────────────────────────────────────────────

    def compute_drone_positions(self):
        """
        Given the virtual midpoint and current heading, calculate both
        drone positions (perpendicular offset left/right of heading).

          Drone 1 ←── separation/2 ──[MIDPOINT]── separation/2 ──→ Drone 2

        Returns ((lat1,lon1,alt), (lat2,lon2,alt))
        """
        # Bearing perpendicular to heading (90° clockwise = right side)
        right_bearing = (self.heading + 90.0) % 360.0
        left_bearing  = (self.heading - 90.0) % 360.0

        half = self.separation / 2.0

        lat1, lon1 = offset_gps(self.mid_lat, self.mid_lon,
                                 right_bearing, half)
        lat2, lon2 = offset_gps(self.mid_lat, self.mid_lon,
                                 left_bearing,  half)

        return (lat1, lon1, self.mid_alt), (lat2, lon2, self.mid_alt)

    # ── Main control loop ─────────────────────────────────────────────────

    def run(self):
        """Main formation control loop. Runs at UPDATE_HZ."""
        dt   = 1.0 / UPDATE_HZ
        iter_count = 0

        self._running = True
        self._telem_thread = threading.Thread(target=self._telem_loop, daemon=True)
        self._telem_thread.start()

        print("\n" + "="*60)
        print("FORMATION CONTROLLER RUNNING")
        print("  Right stick → move midpoint (forward/back/strafe)")
        print("  Left stick  → altitude (up/down) + heading (rotate)")
        print("  VRA knob    → separation distance")
        print("  Ctrl+C      → emergency RTL and exit")
        print("="*60 + "\n")

        try:
            while self._running:
                t0 = time.time()

                # ── Read joystick ──
                joy = self.read_joystick()

                # Update separation from knob (only if knob axis exists)
                if self.joy.get_numaxes() > AXIS_SEP_KNOB:
                    self.separation = joy['separation']

                # ── Update heading ──
                self.heading = (self.heading +
                                joy['yaw'] * MAX_YAW_RATE_DPS * dt) % 360.0

                # ── Update midpoint position ──
                # Convert joystick pitch/roll to North/East velocity (m/s)
                h_rad = math.radians(self.heading)
                # pitch = forward along heading, roll = strafe perpendicular
                v_north = (joy['pitch'] * math.cos(h_rad) -
                           joy['roll']  * math.sin(h_rad)) * MAX_SPEED_H_MS
                v_east  = (joy['pitch'] * math.sin(h_rad) +
                           joy['roll']  * math.cos(h_rad)) * MAX_SPEED_H_MS
                v_up    = joy['throttle'] * MAX_SPEED_V_MS

                # Convert m/s → degrees/s for lat/lon integration
                lat_rad = math.radians(self.mid_lat)
                d_lat   = v_north / 111111.0 * dt
                d_lon   = v_east  / (111111.0 * math.cos(lat_rad)) * dt
                d_alt   = v_up * dt

                self.mid_lat += d_lat
                self.mid_lon += d_lon
                self.mid_alt  = clamp(self.mid_alt + d_alt, MIN_ALT_M, MAX_ALT_M)

                # ── Compute + send positions ──
                pos1, pos2 = self.compute_drone_positions()
                self.d1.send_position(*pos1)
                self.d2.send_position(*pos2)

                # ── Status line ──
                iter_count += 1
                if iter_count % UPDATE_HZ == 0:   # print once per second
                    dist = (haversine_m(self.d1.lat or self.mid_lat,
                                        self.d1.lon or self.mid_lon,
                                        self.d2.lat or self.mid_lat,
                                        self.d2.lon or self.mid_lon)
                            if self.d1.lat and self.d2.lat else 0.0)
                    print(
                        f"  Mid {self.mid_lat:.5f},{self.mid_lon:.5f} "
                        f"Alt={self.mid_alt:.1f}m  Hdg={self.heading:.0f}°  "
                        f"Sep={self.separation:.1f}m  ActualDist={dist:.1f}m"
                    )

                # ── Rate limit ──
                elapsed = time.time() - t0
                sleep_t = dt - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)

        except KeyboardInterrupt:
            print("\n\n[CTRL+C] Emergency RTL triggered …")
        finally:
            self.emergency_rtl()

    def emergency_rtl(self):
        """Send both drones to RTL mode immediately."""
        self._running = False
        print("Sending RTL to both drones …")
        self.d1.land()
        self.d2.land()
        print("RTL commands sent. Monitor landing in Mission Planner.")
        pygame.quit()


# ─────────────────────────── CLI ENTRY POINT ──────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Dual-drone formation controller",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    p.add_argument('--port1',    default=DRONE1_PORT,  help='Serial port for Drone 1')
    p.add_argument('--port2',    default=DRONE2_PORT,  help='Serial port for Drone 2')
    p.add_argument('--baud',     type=int, default=BAUD_RATE, help='Telemetry baud rate')
    p.add_argument('--sep',      type=float, default=DEFAULT_SEP_M,
                   help='Initial separation (m)')
    p.add_argument('--alt',      type=float, default=DEFAULT_ALT_M,
                   help='Target altitude (m AGL)')
    p.add_argument('--arm',      action='store_true',
                   help='Auto-arm and take off on launch (USE CAREFULLY)')
    p.add_argument('--sim',      action='store_true',
                   help='Simulation mode: print commands instead of sending')
    return p.parse_args()


def main():
    args = parse_args()

    print("="*60)
    print("  DUAL-DRONE FORMATION CONTROLLER")
    print("  ArduPilot / MAVLink / RadioMaster TX16S")
    print("="*60)

    # ── Simulation mode (no hardware needed) ──
    if args.sim:
        print("\n[SIM MODE] No drone connections made. Joystick test only.\n")
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("No joystick found.")
            return
        joy = pygame.joystick.Joystick(0)
        joy.init()
        print(f"Joystick: {joy.get_name()}")
        print("Move sticks — press Ctrl+C to exit\n")
        mid_lat, mid_lon, mid_alt = 52.0, 4.0, 8.0
        heading, separation = 0.0, args.sep
        dt = 1.0 / UPDATE_HZ
        try:
            while True:
                pygame.event.pump()
                n = joy.get_numaxes()
                def ax(i): return deadband(joy.get_axis(i)) if i < n else 0.0
                roll = ax(0); pitch = -ax(1); throttle = -ax(2); yaw = ax(3)
                heading = (heading + yaw * MAX_YAW_RATE_DPS * dt) % 360.0
                h_rad = math.radians(heading)
                v_n = (pitch * math.cos(h_rad) - roll * math.sin(h_rad)) * MAX_SPEED_H_MS
                v_e = (pitch * math.sin(h_rad) + roll * math.cos(h_rad)) * MAX_SPEED_H_MS
                mid_lat += v_n / 111111.0 * dt
                mid_lon += v_e / (111111.0 * math.cos(math.radians(mid_lat))) * dt
                mid_alt = clamp(mid_alt + throttle * MAX_SPEED_V_MS * dt, MIN_ALT_M, MAX_ALT_M)
                p_right = (heading + 90) % 360
                p_left  = (heading - 90) % 360
                lat1, lon1 = offset_gps(mid_lat, mid_lon, p_right, separation/2)
                lat2, lon2 = offset_gps(mid_lat, mid_lon, p_left,  separation/2)
                print(f"\r  Hdg={heading:5.1f}°  Alt={mid_alt:4.1f}m  Sep={separation:.1f}m"
                      f"  D1=({lat1:.5f},{lon1:.5f})  D2=({lat2:.5f},{lon2:.5f})", end='')
                time.sleep(dt)
        except KeyboardInterrupt:
            print("\nSim ended.")
        pygame.quit()
        return

    # ── Real hardware mode ──
    drone1 = DroneLink(args.port1, args.baud, DRONE1_SYSID, "D1")
    drone2 = DroneLink(args.port2, args.baud, DRONE2_SYSID, "D2")

    if not drone1.connect():
        print("Failed to connect Drone 1. Check port and power.")
        sys.exit(1)
    if not drone2.connect():
        print("Failed to connect Drone 2. Check port and power.")
        sys.exit(1)

    # Request position telemetry at 5 Hz from both drones
    drone1.request_message_rate(33, 5)  # GLOBAL_POSITION_INT
    drone2.request_message_rate(33, 5)

    fc = FormationController(drone1, drone2, separation=args.sep)
    fc.mid_alt = args.alt

    if not fc.setup_joystick():
        sys.exit(1)

    fc.init_midpoint()

    if args.arm:
        print("\n⚠  AUTO-ARM requested. Arming in 5 seconds … (Ctrl+C to abort)")
        for i in range(5, 0, -1):
            print(f"  {i} …"); time.sleep(1)
        drone1.set_mode('GUIDED')
        drone2.set_mode('GUIDED')
        time.sleep(1)
        drone1.arm(True)
        drone2.arm(True)
        time.sleep(2)
        drone1.takeoff(args.alt)
        drone2.takeoff(args.alt)
        print(f"Takeoff to {args.alt} m commanded. Waiting for climb …")
        time.sleep(8)

    fc.run()


if __name__ == '__main__':
    main()
