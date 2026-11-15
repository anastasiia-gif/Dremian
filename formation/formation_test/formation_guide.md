# Dual-Drone Formation Flying System
## Complete Prototype Guide — ArduPilot + RadioMaster TX16S

**Project goal:** Two ArduPilot copters fly side-by-side at the same altitude, separated
by a configurable distance, with cloth/net strung between them to catch objects in the air.
The RadioMaster TX16S (connected to a laptop via USB) controls a virtual midpoint —
both drones move in formation as if they were opposite ends of a rigid stick.

---

## Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Hardware Requirements](#2-hardware-requirements)
3. [ArduPilot Parameter Configuration](#3-ardupilot-parameter-configuration)
4. [Software Installation](#4-software-installation)
5. [RadioMaster TX16S USB Setup](#5-radiomaster-tx16s-usb-setup)
6. [Understanding the Control Geometry](#6-understanding-the-control-geometry)
7. [Step-by-Step Testing Procedure](#7-step-by-step-testing-procedure)
8. [Running the Formation Script](#8-running-the-formation-script)
9. [Telemetry & Monitoring](#9-telemetry--monitoring)
10. [Net/Cloth Integration](#10-netcloth-integration)
11. [Troubleshooting](#11-troubleshooting)
12. [Safety Rules](#12-safety-rules)

---

## 1. System Architecture

```
┌───────────────────────────────────────────────────────────┐
│                         LAPTOP                            │
│                                                           │
│  ┌──────────────┐    ┌───────────────────────────────┐   │
│  │ RadioMaster  │    │   formation_controller.py      │   │
│  │  TX16S USB   │───▶│                               │   │
│  │  (Joystick)  │    │  Virtual Midpoint Engine      │   │
│  └──────────────┘    │  Heading, Alt, Sep calculator │   │
│                      │                               │   │
│                      │  D1 position  D2 position     │   │
│                      └──────┬────────────┬───────────┘   │
│                             │            │               │
│                    [COM3]   │            │   [COM4]       │
└────────────────────────┬────┘            └────┬──────────┘
                         │                      │
                    [USB/Serial]           [USB/Serial]
                         │                      │
               ┌─────────▼──┐          ┌────────▼──────┐
               │ SiK Radio  │          │  SiK Radio    │
               │ Ground #1  │          │  Ground #2    │
               │ NetID = 25 │          │  NetID = 26   │
               └─────────┬──┘          └────────┬──────┘
                         │  433 MHz RF            │  433 MHz RF
               ┌─────────▼──┐          ┌─────────▼──────┐
               │ SiK Radio  │          │  SiK Radio     │
               │  Air #1    │          │   Air #2       │
               │ NetID = 25 │          │  NetID = 26    │
               └─────────┬──┘          └─────────┬──────┘
                         │                        │
               ┌─────────▼──────┐      ┌──────────▼─────┐
               │   DRONE 1      │      │    DRONE 2      │
               │ ArduPilot FC   │      │  ArduPilot FC   │
               │ SYSID_THISMAV=1│      │ SYSID_THISMAV=2 │
               │ GUIDED mode    │      │  GUIDED mode    │
               └────────────────┘      └─────────────────┘
                    ╲                            ╱
                     ╲______ NET/CLOTH __________╱
```

**Key points:**
- Two completely separate telemetry radio pairs (different NET_ID)
- Script reads the TX16S as a USB HID joystick via `pygame`
- Commands sent via MAVLink `SET_POSITION_TARGET_GLOBAL_INT`
- Each drone runs in **GUIDED mode** — the flight controller handles all
  stabilisation; the script only updates target GPS positions at 10 Hz
- The RC receivers on the drones are NOT used during formation flight

---

## 2. Hardware Requirements

### Drones
- 2× ArduCopter drones (any frame size ≥ 250 mm recommended)
  - Minimum wheelbase for net mission: 350 mm (7" props or larger)
- GPS module on each (M8N or better recommended)
- ArduPilot firmware (latest stable, e.g. 4.5.x)

### Telemetry
- 2× pairs of SiK 433 MHz (or 915 MHz) telemetry radios
  - Pair A: Air unit on Drone 1 + Ground unit to laptop COM3
  - Pair B: Air unit on Drone 2 + Ground unit to laptop COM4
  - Each pair MUST have a unique NET_ID so they don't interfere

### Controller
- RadioMaster TX16S (OpenTX / EdgeTX, any variant)
- USB-C to USB-A (or USB-C to USB-C) cable, **data-capable** (not just charge)

### Laptop
- Windows 10/11, macOS, or Linux
- 2× available USB ports (or USB hub)
- Python 3.10+

---

## 3. ArduPilot Parameter Configuration

**Do this step first, before any flight.** Connect to each drone individually
in Mission Planner and set these parameters. Click **Write Params** after each drone.

### Drone 1 Parameters

| Parameter | Value | Notes |
|---|---|---|
| `SYSID_THISMAV` | **1** | Unique system ID |
| `SERIAL1_PROTOCOL` | 2 | MAVLink2 on Telem1 |
| `SERIAL1_BAUD` | 57 | 57600 baud |
| `GUID_OPTIONS` | 0 | Default GUIDED behaviour |
| `WP_SPEED` | 300 | 3 m/s max speed in GUIDED (cm/s) |
| `WPNAV_ACCEL` | 100 | Smooth acceleration (cm/s²) |
| `RTL_ALT` | 1500 | RTL climb to 15 m before returning (cm) |
| `FS_THR_ENABLE` | 1 | Throttle failsafe → RTL |
| `FS_GCS_ENABLE` | 1 | GCS failsafe (if telemetry lost → RTL) |
| `FENCE_ENABLE` | 1 | Enable geofence |
| `FENCE_TYPE` | 4 | Circle fence |
| `FENCE_RADIUS` | 50 | 50 m radius safety bubble |
| `FENCE_ALT_MAX` | 30 | 30 m altitude ceiling |
| `FENCE_ACTION` | 1 | Fence breach → RTL |

### Drone 2 Parameters
Same as above, except:

| Parameter | Value |
|---|---|
| `SYSID_THISMAV` | **2** |

### SiK Radio Configuration (Mission Planner → Initial Setup → SiK Radio)
Connect each radio pair individually and set:

| Parameter | Pair A (Drone 1) | Pair B (Drone 2) |
|---|---|---|
| NET_ID | 25 | 26 |
| Baud | 57600 | 57600 |
| Air Speed | 64 | 64 |
| ECC | enabled | enabled |
| Mavlink | MAVLink | MAVLink |

Click **Save Settings** and **Copy Required Items to Remote** for each pair.

> ⚠️ **Critical:** Both radios in each pair must have the same NET_ID.
> The two pairs must have *different* NET_IDs so they don't hear each other.

---

## 4. Software Installation

### Install Python dependencies

```bash
pip install pymavlink pygame
```

Verify:
```bash
python -c "from pymavlink import mavutil; print('pymavlink OK')"
python -c "import pygame; print('pygame OK')"
```

### Find your serial ports

**MissionPlanner:**
- After refresh should appear in COM ports as new available ports

**Windows:**
- Open Device Manager → Ports (COM & LPT)
- Plug/unplug each radio to identify its COM number
- They will show as "Silicon Labs CP210x" or "USB Serial Device"

**Linux:**
```bash
ls /dev/ttyUSB*   # SiK radios usually appear as ttyUSB0, ttyUSB1
```
Grant permission (once):
```bash
sudo usermod -aG dialout $USER
# Log out and back in for it to take effect
```

**macOS:**
```bash
ls /dev/cu.usbserial-*
```

### Edit the script configuration block

Open `formation_controller.py` and update the top section:
```python
DRONE1_PORT  = 'COM3'   # ← Your actual port for Drone 1
DRONE2_PORT  = 'COM4'   # ← Your actual port for Drone 2
```

---

## 5. RadioMaster TX16S USB Setup

### Step 1 — Enable Joystick mode in EdgeTX/OpenTX

1. Power on the TX16S
2. Press `SYS` (or `MDL` depending on firmware)
3. Go to **Radio Setup** → **USB Mode** → set to **Ask**
4. Save and exit

### Step 2 — Connect via the TOP USB-C port

There are two USB-C ports on the TX16S:
- **Bottom**: Charging only
- **Top** (under the antenna flap): Data / PC connection ← use this one

Connect top port → laptop. The TX16S screen will ask:
> `USB Joystick` or `USB Storage`

Select **USB Joystick**.

### Step 3 — Verify on Windows

- Start → type "Game Controllers" → "Set up USB Game Controllers"
- "RadioMaster TX16S" (or similar) should appear
- Click Properties → Test tab → move sticks to verify axes respond

### Step 4 — Map your sticks (run the axis mapper)

Run this quick script to see your axis numbers:
```python
import pygame, time
pygame.init(); pygame.joystick.init()
j = pygame.joystick.Joystick(0); j.init()
while True:
    pygame.event.pump()
    axes = [round(j.get_axis(i),2) for i in range(j.get_numaxes())]
    print(f"\r{axes}", end='')
    time.sleep(0.1)
```
Move each stick and note which axis index changes.
Update `AXIS_ROLL`, `AXIS_PITCH`, `AXIS_THROTTLE`, `AXIS_YAW` in the script if needed.

### Windows driver fix (if TX16S not detected as joystick)

If it appears as "BETTER_USB_HS" rather than a joystick:
1. Device Manager → locate BETTER_USB_HS
2. Right-click → Update Driver → Browse → Let me pick
3. Choose **USB Input Device** (NOT libusb-win32)
4. Click Next, then Close

---

## 6. Understanding the Control Geometry

### The "Stick" Formation

```
Flight direction →
                    ╔═════════╗
Drone 1 ────────────╢  cloth  ╟──────────── Drone 2
  (left side)       ╚═════════╝              (right side)
         ←  sep/2  →  [MID]  ←  sep/2  →

        heading = 0° (North)
        separation = 4 m
        both drones at same altitude
```

### How the virtual midpoint works

1. **Joystick pitch** (right stick up) → midpoint moves forward along current heading
2. **Joystick roll** (right stick right) → midpoint strafes right
3. **Left stick up** → midpoint rises in altitude
4. **Left stick L/R** → heading of the stick rotates
5. **VRA knob** → changes separation distance live

### What happens when you turn

When you yaw the heading by 90°:

```
Before (heading=0°, flying North):

   D1 ──── cloth ──── D2
              ↑ heading

After (heading=90°, flying East):

              ↑ D2
              │
            cloth
              │
              ↓ D1
         heading →
```

Both drones stay perpendicular to the heading. The cloth always faces the direction of travel.

### Coordinate math (simplified)

```
perpendicular_bearing = heading + 90°

D1_position = offset(midpoint, perp_bearing,      sep/2)
D2_position = offset(midpoint, perp_bearing+180°, sep/2)
```

The script updates these positions at **10 Hz** (every 100 ms) and sends
`SET_POSITION_TARGET_GLOBAL_INT` to each drone individually.

---

## 7. Step-by-Step Testing Procedure

### Phase 0 — Simulation Test (no drones, no props)

Test the joystick and geometry math only.

```bash
python formation_controller.py --sim
```

- Move all sticks and verify the printed coordinates change correctly
- Check that roll/pitch/yaw/altitude all respond as expected
- Verify separation knob changes the sep value
- Expected output: `Hdg=  0.0° Alt= 8.0m Sep=3.0m D1=(...) D2=(...)`

**Pass criteria:** All axes respond, no crashes, geometry looks sensible.

---

### Phase 1 — Bench Test (motors OFF, props OFF)

**Setup:** Drones on table, powered, no props, telemetry radios connected.

#### Step 1.1 — Verify telemetry connections
```bash
python -c "
from pymavlink import mavutil
c = mavutil.mavlink_connection('COM3', baud=57600)
hb = c.wait_heartbeat(timeout=10)
print('DRONE 1:', hb)
"
```
Repeat for COM4 / Drone 2.

#### Step 1.2 — Verify unique SYSIDs
Open Mission Planner → connect to each drone → Full Parameter List → check `SYSID_THISMAV`.
Drone 1 must be **1**, Drone 2 must be **2**.

#### Step 1.3 — Verify GUIDED mode command acceptance
```bash
# In Mission Planner, right-click map → "Fly to Here" → it should accept
# This confirms GUIDED mode works and GPS lock is present
```

#### Step 1.4 — Run the script in dry-run
```bash
python formation_controller.py --port1 COM3 --port2 COM4
```
- Both drones should show "Connected ✓"
- The midpoint should initialise from current GPS positions
- Move sticks: formation positions should print to terminal
- Verify separation knob changes printed sep distance

**Pass criteria:** Both drones connect, positions compute correctly, no errors.

---

### Phase 2 — Tethered Hover Test

**Setup:** Open area, drones tethered with ~5 m light string to ground anchors,
**minimum 10 m apart** from each other, props installed, safety pilot standing by.

> ⚠️ **IMPORTANT:** Have a human safety pilot with a second RC transmitter
> bound to each drone (trainer mode or separate bind). Safety pilot can override
> at any moment by taking stick control or switching to LOITER.

#### Step 2.1 — Pre-flight checklist (for EACH drone)
- [ ] Props tight and correct orientation
- [ ] Battery fully charged
- [ ] GPS lock acquired (solid blue/green LED, 8+ satellites)
- [ ] All arming checks pass in Mission Planner
- [ ] Tether attached to frame, NOT to props
- [ ] Safety pilot transmitter bound and in LOITER mode

#### Step 2.2 — Arm and takeoff manually first
1. Switch Drone 1 to LOITER, arm manually via RC, takeoff to 3 m
2. Switch Drone 2 to LOITER, arm manually via RC, takeoff to 3 m
3. Verify stable hover on both
4. Land both manually

#### Step 2.3 — Test GUIDED mode individually
1. Open Mission Planner for Drone 1
2. Set mode to GUIDED
3. Right-click on map 3 m away → "Fly to Here"
4. Drone 1 should fly to that position
5. Repeat for Drone 2
6. Land both

#### Step 2.4 — First formation test (low altitude, short duration)
1. Arm both drones manually
2. Take off to ~3 m manually
3. Switch both to GUIDED mode
4. Run the formation script (script should detect they're already airborne)
5. Make VERY SMALL stick inputs (10–20% deflection)
6. Observe both drones responding
7. Test: small forward movement, small rotation, altitude up/down
8. Run for 60 seconds maximum
9. Ctrl+C → both drones go to RTL

**Pass criteria:** Both drones follow formation commands, maintain relative separation,
RTL on Ctrl+C.

---

### Phase 3 — Open Field Formation Flight

**Setup:** Open field, at least 50×50 m clear area, no tethers.

#### Step 3.1 — Gradual expansion test
Start with `--sep 2.0` (2 m separation), verify behaviour, then increase to 3, 4, 5 m.

```bash
python formation_controller.py --port1 COM3 --port2 COM4 --alt 8 --sep 2.0
```

#### Step 3.2 — Manoeuvre sequence (do in order)
1. **Hover hold** (5 min): No stick input. Verify drones hold position.
2. **Forward flight** (50 m): Gentle forward stick, low speed (1–2 m/s)
3. **Strafe left/right** (20 m): Right stick L/R
4. **Altitude change**: Left stick up/down, +/−3 m from baseline
5. **Heading rotation** (45°): Left stick L/R, slow yaw
6. **Combined manoeuvre**: Forward + gentle yaw turn (like flying a curve)
7. **Return and land**

#### Step 3.3 — Separation test
- While hovering, turn the VRA knob from minimum to maximum slowly
- Observe both drones separating/closing while maintaining midpoint
- Minimum safe test: 1.5 m / Maximum for net test: depends on net size

**Pass criteria:** Smooth formation flight, no oscillations, clean RTL.

---

### Phase 4 — Net/Cloth Integration Test

See Section 10 for cloth attachment procedure. Only proceed here after Phase 3 passes.

---

## 8. Running the Formation Script

### Basic usage

```bash
# Start (manual arm via RC or Mission Planner beforehand)
python formation_controller.py --port1 COM3 --port2 COM4

# Custom separation and altitude
python formation_controller.py --port1 COM3 --port2 COM4 --alt 10 --sep 4.0

# Auto-arm and takeoff (USE ONLY after extensive manual testing)
python formation_controller.py --port1 COM3 --port2 COM4 --arm

# Simulation (no drones needed)
python formation_controller.py --sim
```

### Command-line options

| Flag | Default | Description |
|---|---|---|
| `--port1` | COM3 | Serial port for Drone 1's telemetry radio |
| `--port2` | COM4 | Serial port for Drone 2's telemetry radio |
| `--baud` | 57600 | Telemetry baud rate |
| `--sep` | 3.0 | Initial separation in metres |
| `--alt` | 8.0 | Target altitude in metres AGL |
| `--arm` | off | Auto-arm and takeoff on start |
| `--sim` | off | Simulation mode (no hardware) |

### Stick mapping reference

| Stick | Axis | Formation Effect |
|---|---|---|
| Right stick ↑↓ | Pitch | Move midpoint forward/backward |
| Right stick ←→ | Roll | Strafe midpoint left/right |
| Left stick ↑↓ | Throttle | Change altitude |
| Left stick ←→ | Yaw | Rotate formation heading |
| VRA knob | Axis 4 | Separation distance (1.5–10 m) |

---

## 9. Telemetry & Monitoring

### Monitor both drones simultaneously in Mission Planner

1. Open Mission Planner
2. Click **View** → **Multiple Mavlink** (or use MAVProxy for advanced use)
3. Connect to COM3 (Drone 1) → it will appear on map
4. Click **+** to add a second connection → COM4 (Drone 2)
5. Both should appear on map simultaneously

### What to watch during formation flight

| Metric | Acceptable | Warning |
|---|---|---|
| GPS HDOP | < 1.5 | > 2.0 (position accuracy degraded) |
| GPS satellites | ≥ 10 | < 8 |
| Battery voltage | > 3.7V/cell | < 3.5V/cell → land |
| Actual drone separation | within 0.5 m of commanded | > 1 m off → land |
| Flight mode | GUIDED | Any other mode → investigate |
| Radio RSSI | > -90 dBm | < -100 dBm → risk of loss |

### Optional: Add telemetry logging to the script

Uncomment or add in the `run()` loop:
```python
import csv
with open('formation_log.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['time','mid_lat','mid_lon','mid_alt','heading','separation',
                     'd1_lat','d1_lon','d2_lat','d2_lon'])
    # inside loop:
    writer.writerow([time.time(), self.mid_lat, self.mid_lon, self.mid_alt,
                     self.heading, self.separation,
                     self.d1.lat, self.d1.lon, self.d2.lat, self.d2.lon])
```

---

## 10. Net/Cloth Integration

### Cloth selection

| Property | Requirement |
|---|---|
| Weight | As light as possible — budget < 200 g total for initial test |
| Porosity | High (mesh/net preferred to solid cloth) — reduces drag and wind effects |
| Width | Span - 0.5 m (leave 0.25 m margin per side) |
| Attachment | Loops or clips, NOT tied to props or motor mounts |

### Attachment method

**DO NOT attach cloth directly to the frame.** Use dedicated attachment points:

1. Mount a small lightweight spreader bar (3 mm carbon tube or bamboo) horizontally under each drone, perpendicular to the drone body, at the **centre of gravity height**
2. Attach the cloth/net to the spreader bar ends using carabiner clips or velcro loops
3. The attachment point must be **below** the prop wash zone
4. Use a bungee/elastic section in each attachment to absorb tugs

```
      [Drone 1]                    [Drone 2]
          │                            │
     [spreader bar]              [spreader bar]
      ↙        ↘                ↙         ↘
  loop         loop-─────net─────-loop       loop
```

### Physics considerations

When cloth is attached, the drones will experience:
- **Drag**: proportional to cloth area × wind speed²
  - Fly slower (< 2 m/s) and into the wind when cloth is attached
- **Tension**: if drones drift apart beyond the cloth's natural span, the cloth will pull them together — this is a **feedback loop** that helps but can also cause oscillations
- **Wind gusts**: the cloth acts as a sail. Reduce separation when wind > 3 m/s.

### Recommended net test sequence

1. First test: **static hover only**, no movement, just verify cloth hangs correctly
2. Second test: **slow vertical movement** (1 m/s max)
3. Third test: **slow forward flight** (< 1 m/s) — watch for pitch disturbance
4. Fourth test: **increase speed gradually** — note max stable speed with this cloth
5. Catching test: have a helper gently drop a light object (foam ball) into the net

### Separation for net deployment

| Net width | Recommended separation | Notes |
|---|---|---|
| 1 m | 1.8 m | Tight — cloth will have tension |
| 2 m | 2.5 m | Slight sag, natural catenary shape |
| 3 m | 4.0 m | Good for catching — slight V shape |
| 5 m | 6.0 m | Large catch area — more drag |

---

## 11. Troubleshooting

### Drone won't enter GUIDED mode
- Check GPS lock: needs ≥ 6 satellites, HDOP < 2.0
- EKF must be healthy (green bar in Mission Planner HUD)
- Pre-arm checks must all pass

### Script connects but position commands have no effect
- Verify drone is in GUIDED mode BEFORE sending position commands
- Check `SYSID_THISMAV` matches `DRONE1_SYSID`/`DRONE2_SYSID` in the script
- Try resending with `target_system=0` (broadcast) to confirm message is received

### Drones oscillate / overshoot targets
- Reduce `WP_SPEED` in ArduPilot parameters (start at 100–200 cm/s)
- Reduce `WPNAV_ACCEL` to 50 cm/s²
- Reduce `MAX_SPEED_H_MS` in the script to 1.5 m/s

### TX16S not detected as joystick on Windows
- Use the **top** USB-C port (not the charging port)
- Select "USB Joystick" on the TX16S screen when prompted
- If BETTER_USB_HS appears: Device Manager → Update Driver → USB Input Device
- Try a different USB cable (power-only cables won't work)

### Two radios interfering with each other
- Verify different NET_ID on each pair (one pair = 25, other = 26)
- Use Mission Planner's SiK Radio setup to verify and save settings
- Physical separation of ground units helps (> 0.5 m apart on your table)

### Drone 1 receiving Drone 2's MAVLink messages
- This can happen if radios share the same NET_ID
- The routing is handled by `target_system` ID, but separate NET_IDs is cleaner

### GPS accuracy poor (drones drift apart while hovering)
- Use a clear outdoor area, away from buildings
- Wait until HDOP < 1.5 before arming
- Consider RTK GPS for better accuracy (required for < 0.5 m separation)

---

## 12. Safety Rules

### ALWAYS
- [ ] Have a dedicated safety pilot with RC override ability for each drone
- [ ] Perform pre-flight checks before every flight
- [ ] Test in simulation mode before any real flight
- [ ] Start with minimum separation (2 m) and increase gradually
- [ ] Keep initial tests below 5 m altitude
- [ ] Have emergency LAND/RTL switch accessible at all times
- [ ] Keep spectators at least 20 m away
- [ ] Fly in calm conditions (wind < 5 m/s without net, < 3 m/s with net)

### NEVER
- [ ] Do not arm if HDOP > 2.0
- [ ] Do not fly if battery < 30%
- [ ] Do not fly within 5 km of an airport without permission
- [ ] Do not fly over people
- [ ] Do not exceed local altitude limits
- [ ] Do not fly with broken/worn props
- [ ] Do not ignore EKF warnings

### Emergency procedures

| Situation | Action |
|---|---|
| Script crashes | Both drones should RTL (GCS failsafe) within `FS_GCS_TIMEOUT` seconds |
| Drone drifts unexpectedly | Safety pilot takes over, switch to LOITER |
| Cloth entangles props | Safety pilot lands both immediately, slowly |
| Radio link lost | Drone should auto-RTL (FS_GCS_ENABLE = 1) |
| Drone descends unexpectedly | Switch to ALT_HOLD or STABILIZE, land manually |

---

## Appendix A — Complete Parameter Reference

```
# Both drones (change SYSID per drone)
SYSID_THISMAV    = 1 (or 2)
SERIAL1_PROTOCOL = 2
SERIAL1_BAUD     = 57
GUID_OPTIONS     = 0
WP_SPEED         = 300
WPNAV_ACCEL      = 100
RTL_ALT          = 1500
FS_THR_ENABLE    = 1
FS_GCS_ENABLE    = 1
FS_GCS_TIMEOUT   = 5
FENCE_ENABLE     = 1
FENCE_TYPE       = 4
FENCE_RADIUS     = 50
FENCE_ALT_MAX    = 30
FENCE_ACTION     = 1
```

## Appendix B — Quick Start Checklist

```
□ Both drones have unique SYSID_THISMAV (1 and 2)
□ SiK radio pairs have different NET_IDs (25 and 26)
□ Python packages installed (pymavlink, pygame)
□ Serial ports identified (COM3, COM4)
□ Script DRONE1_PORT / DRONE2_PORT updated
□ TX16S in USB Joystick mode (top USB-C port)
□ Axis mapping verified with --sim mode
□ Bench test passed (Phase 1)
□ Tethered test passed (Phase 2)
□ Open field test passed (Phase 3)
□ Ready for net integration (Phase 4)
```

---

*Guide version 1.0 — For ArduCopter 4.4+ / OpenTX 2.3+ / EdgeTX 2.9+*
