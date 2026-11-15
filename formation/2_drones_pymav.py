import time
import math
import sys
from pymavlink import mavutil

# ── Config — update before flying ─────────────────────────────
DRONE1_PORT = "udp:0.0.0.0:14551"
DRONE2_PORT = "udp:0.0.0.0:14561"

TAKEOFF_ALT  = 10.0    # metres
HOVER_TIME   = 10.0    # seconds to hover at target before asking
OFFSET_M     = 5.0     # lateral offset from target centre (net span/2)

# Update these to your actual field GPS coordinates
TARGET_LAT   = 52.012345
TARGET_LON   =  4.123456

# ── Helpers ────────────────────────────────────────────────────
def connect(port, name):
    print(f"[{name}] Connecting to {port}...")
    v = mavutil.mavlink_connection(port)
    v.wait_heartbeat(timeout=30)
    print(f"[{name}] Connected! sysid={v.target_system}")
    return v

def get_pos(v, timeout=5):
    msg = v.recv_match(type='GLOBAL_POSITION_INT',
                       blocking=True, timeout=timeout)
    if msg:
        return (msg.lat/1e7, msg.lon/1e7,
                msg.relative_alt/1000.0)
    return None

def get_battery(v, timeout=3):
    msg = v.recv_match(type='SYS_STATUS',
                       blocking=True, timeout=timeout)
    if msg:
        return msg.battery_remaining
    return -1

def set_guided(v, name):
    v.mav.command_long_send(
        v.target_system, v.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4, 0, 0, 0, 0, 0)
    print(f"[{name}] GUIDED mode set")

def arm(v, name):
    v.mav.command_long_send(
        v.target_system, v.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    print(f"[{name}] Armed")

def disarm(v, name):
    v.mav.command_long_send(
        v.target_system, v.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 21196, 0, 0, 0, 0, 0)
    print(f"[{name}] Disarmed")

def takeoff(v, name, alt):
    v.mav.command_long_send(
        v.target_system, v.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt)
    print(f"[{name}] Takeoff to {alt}m commanded")

def land(v, name):
    v.mav.command_long_send(
        v.target_system, v.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)
    print(f"[{name}] Land commanded")

def goto(v, name, lat, lon, alt):
    v.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            0, v.target_system, v.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b110111111000,
            int(lat*1e7), int(lon*1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0))
    print(f"[{name}] Going to {lat:.6f},{lon:.6f} @ {alt}m")

def dist_m(lat1, lon1, lat2, lon2):
    dlat = (lat2-lat1) * 111320
    dlon = (lon2-lon1) * 111320 * math.cos(math.radians(lat1))
    return math.sqrt(dlat**2 + dlon**2)

def wait_alt(v, name, target, tol=2.0, timeout=40):
    print(f"[{name}] Climbing to {target}m...")
    t0 = time.time()
    while time.time()-t0 < timeout:
        pos = get_pos(v)
        if pos:
            print(f"[{name}]   alt={pos[2]:.1f}m")
            if abs(pos[2]-target) < tol:
                return True
        time.sleep(1.5)
    print(f"[{name}] WARNING — altitude timeout!")
    return False

def wait_pos(v, name, lat, lon, radius=4.0, timeout=90):
    print(f"[{name}] Flying to position...")
    t0 = time.time()
    while time.time()-t0 < timeout:
        pos = get_pos(v)
        if pos:
            d = dist_m(pos[0], pos[1], lat, lon)
            print(f"[{name}]   dist={d:.1f}m  alt={pos[2]:.1f}m")
            if d < radius:
                print(f"[{name}] Position reached!")
                return True
        time.sleep(2)
    print(f"[{name}] WARNING — position timeout!")
    return False

def wait_landed(v, name, timeout=30):
    print(f"[{name}] Waiting for landing...")
    t0 = time.time()
    while time.time()-t0 < timeout:
        pos = get_pos(v)
        if pos:
            print(f"[{name}]   alt={pos[2]:.1f}m")
            if pos[2] < 0.5:
                print(f"[{name}] Landed!")
                return True
        time.sleep(1)
    return False

def confirm(prompt):
    """Pause and wait for operator to press Enter."""
    print(f"\n{'='*55}")
    print(f"  {prompt}")
    print(f"  Press ENTER to continue  |  Ctrl+C to ABORT")
    print(f"{'='*55}")
    try:
        input()
    except KeyboardInterrupt:
        print("\nABORTED by operator.")
        sys.exit(1)

def status(d1, d2):
    """Print a quick status of both drones."""
    p1 = get_pos(d1, timeout=2)
    p2 = get_pos(d2, timeout=2)
    b1 = get_battery(d1, timeout=2)
    b2 = get_battery(d2, timeout=2)
    print(f"\n  D1: {p1[0]:.6f},{p1[1]:.6f} alt={p1[2]:.1f}m batt={b1}%"
          if p1 else "  D1: no position")
    print(f"  D2: {p2[0]:.6f},{p2[1]:.6f} alt={p2[2]:.1f}m batt={b2}%"
          if p2 else "  D2: no position")

# ── MAIN MISSION ───────────────────────────────────────────────

print("\n" + "="*55)
print("  DUAL DRONE FIELD TEST")
print("  Target:", TARGET_LAT, TARGET_LON)
print("  Takeoff alt:", TAKEOFF_ALT, "m")
print("  Net offset:", OFFSET_M, "m")
print("="*55 + "\n")

# ── Step 1: Connect ────────────────────────────────────────────
print(">>> STEP 1: Connecting to both drones...")
d1 = connect(DRONE1_PORT, "Drone1")
d2 = connect(DRONE2_PORT, "Drone2")
print("\nBoth drones connected!")
status(d1, d2)

# ── Step 2: Pre-flight checks ──────────────────────────────────
confirm("CHECK: Both drones on ground, props clear, "
        "people behind safety line. Ready to arm?")

# ── Step 3: Set GUIDED and arm both ───────────────────────────
print("\n>>> STEP 3: Setting GUIDED mode and arming...")
set_guided(d1, "Drone1")
set_guided(d2, "Drone2")
time.sleep(1)

arm(d1, "Drone1")
arm(d2, "Drone2")
time.sleep(2)

status(d1, d2)

# ── Step 4: Confirm takeoff ────────────────────────────────────
confirm(f"CONFIRM: Both drones armed and motors spinning. "
        f"Safe to take off to {TAKEOFF_ALT}m?")

# ── Step 5: Takeoff both simultaneously ───────────────────────
print(f"\n>>> STEP 5: Taking off to {TAKEOFF_ALT}m...")
takeoff(d1, "Drone1", TAKEOFF_ALT)
takeoff(d2, "Drone2", TAKEOFF_ALT)

# Wait for both to reach altitude
import threading
alt_ok = [False, False]

def check_alt(v, name, idx):
    alt_ok[idx] = wait_alt(v, name, TAKEOFF_ALT)

t1 = threading.Thread(target=check_alt, args=(d1,"Drone1",0))
t2 = threading.Thread(target=check_alt, args=(d2,"Drone2",1))
t1.start(); t2.start()
t1.join();  t2.join()

if not all(alt_ok):
    print("\nWARNING: One or both drones did not reach altitude!")
    confirm("Override and continue anyway?")

status(d1, d2)

# ── Step 6: Confirm fly to target ─────────────────────────────
offset_deg = OFFSET_M / 111320.0
d1_lat = TARGET_LAT
d1_lon = TARGET_LON - offset_deg   # drone 1 left of target
d2_lat = TARGET_LAT
d2_lon = TARGET_LON + offset_deg   # drone 2 right of target

print(f"\n  D1 will fly to: {d1_lat:.6f}, {d1_lon:.6f}")
print(f"  D2 will fly to: {d2_lat:.6f}, {d2_lon:.6f}")

confirm(f"CONFIRM: Both drones stable at {TAKEOFF_ALT}m. "
        f"Send to target position?")

# ── Step 7: Fly to offset positions ───────────────────────────
print("\n>>> STEP 7: Flying to target positions...")
goto(d1, "Drone1", d1_lat, d1_lon, TAKEOFF_ALT)
goto(d2, "Drone2", d2_lat, d2_lon, TAKEOFF_ALT)

pos_ok = [False, False]

def check_pos(v, name, lat, lon, idx):
    pos_ok[idx] = wait_pos(v, name, lat, lon)

t1 = threading.Thread(target=check_pos,
                       args=(d1,"Drone1",d1_lat,d1_lon,0))
t2 = threading.Thread(target=check_pos,
                       args=(d2,"Drone2",d2_lat,d2_lon,1))
t1.start(); t2.start()
t1.join();  t2.join()

status(d1, d2)

# ── Step 8: Hover and confirm ──────────────────────────────────
print(f"\n>>> STEP 8: Hovering at target for {HOVER_TIME}s...")
for i in range(int(HOVER_TIME)):
    time.sleep(1)
    if i % 3 == 0:
        status(d1, d2)

confirm("CONFIRM: Formation looks good. "
        "Both drones stable at offset. Ready to land?")

# ── Step 9: Land both ──────────────────────────────────────────
print("\n>>> STEP 9: Landing both drones...")
land(d1, "Drone1")
land(d2, "Drone2")

l1 = threading.Thread(target=wait_landed, args=(d1,"Drone1"))
l2 = threading.Thread(target=wait_landed, args=(d2,"Drone2"))
l1.start(); l2.start()
l1.join();  l2.join()

# ── Step 10: Disarm ────────────────────────────────────────────
print("\n>>> STEP 10: Disarming...")
time.sleep(2)
disarm(d1, "Drone1")
disarm(d2, "Drone2")

print("\n" + "="*55)
print("  MISSION COMPLETE")
print("="*55)
status(d1, d2)