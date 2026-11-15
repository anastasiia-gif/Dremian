# Architecture

How the pieces fit, why they're split this way, and how to bring it up on real
hardware. Written for whoever picks this up next — including future us.

## The stack

Each layer talks only to the one below it through a narrow interface, so any layer can
be swapped without touching the rest. Bottom to top:

```
        ┌─────────────────────────────────────────────┐
        │  frontend/index.html                         │  Setup screen + cockpit:
        │  (browser dashboard)                         │  2D plan + 3D view, roster,
        │                                              │  staged-flight controls, pad
        └───────────────▲─────────────────────────────┘
                        │  websocket JSON  (protocol.py: config / state / roster / events)
        ┌───────────────┴─────────────────────────────┐
        │  app.py — Controller + 10 Hz loop            │  orchestration +
        │           + flight-phase sequencer           │  staged take-off/land +
        │           + runtime reconfigure              │  link rebuild (sim ↔ real)
        └──▲──────────▲──────────▲───────────▲─────────┘
           │          │          │           │
   discovery.py   geometry.py  safety.py   protocol.py
   roster /       midpoint →   separation, serialize
   identify /     targets,     net band,
   profiles       ENU↔GPS      fence, clamp
           │
        ┌──┴──────────────────────────────────────────┐
        │  sources/  VehicleLink (base.py)             │  one interface…
        │    ├── sim_source.py    SimLink              │  …two backends
        │    └── mavlink_source.py MAVLinkLink         │
        └──────────────────────────────────────────────┘
```

## What each layer owns

- **`sources/` (the link).** Opens a transport, ingests telemetry, exposes a uniform
  `snapshot() -> {sysid: VehicleState}` and `send_setpoint(sysid, target)`. `SimLink`
  fakes the vehicles; `MAVLinkLink` talks to real autopilots. Nothing above this layer
  imports pymavlink or knows a socket exists.

- **`discovery.py` (Project A).** Turns raw source-system ids into a named, profiled,
  readiness-checked roster. Identity and capabilities come *from the autopilot*
  (HEARTBEAT, AUTOPILOT_VERSION); the physical profile comes *from config*, because the
  FC can't supply it. Fires join/leave callbacks so the loop and UI react without
  polling.

- **`formation/geometry.py`.** The kinematics. One virtual midpoint pose
  `(e, n, u, yaw)` plus a `FormationSpec` (slot offsets in unit spacing) produces a
  global setpoint per drone. Shapes scale with a single `separation_m` knob. Handles
  the local-tangent ENU ↔ lat/lon projection both ways.

- **`formation/safety.py`.** Evaluates the fleet (collision separation, net band,
  geofence, per-vehicle health) into warn/block violations, and — more importantly —
  *clamps* the midpoint and separation so the operator physically can't drive the group
  out of the fence, past the net's safe span, or below the floor.

- **`app.py`.** The control loop (default 10 Hz) wires it together and owns operator
  state (the formation, the midpoint, the stick) **and the flight phase**. It drives the
  staged take-off/landing sequence, can **reconfigure the link at runtime** (swap the
  simulator for real radios, resize the fleet, change profiles/fence — no restart), and
  fans the whole picture out to every dashboard over the websocket.

## Control loop (one tick)

```
snapshot()                       pull current vehicle states from the link
  → discovery.update()           recompute roster, fire join/leave
  → set origin                   first GPS fix defines the shared ENU frame
  → resize spec to the fleet     while on the ground, match slot count to drones
  → assign_slots()               sysid → slot (deterministic, sorted)
  → advance_phase()              step the flight-phase machine (below); drive seq altitude,
                                 spreading, climb/descent; disarm on the ground
  → integrate operator stick     body-frame fwd/right/up/yaw → midpoint (AIRBORNE only)
  → safety.evaluate()            warn/block violations (props-clear aware)
  → if phase is active & safe:
        clamp_separation()       cap to the net span         (formation phases)
        clamp_midpoint()         keep the footprint in fence  (formation phases)
        phase_targets()          per-drone setpoints: hold-horizontal during vertical
                                 phases, formation geometry while spread
        send_setpoint() × N      skip any vehicle under a hard block
  → broadcast state (+ phase)    to all dashboards
  → broadcast roster (on change)
```

## Flight-phase sequencer

A net strung between drones makes simultaneous, unsynchronised vertical motion dangerous:
one drone lifting first drags the cloth into a neighbour's props. The sequencer makes the
group move like one body.

```
disarmed → arming → ascending → spreading → climbing → AIRBORNE
                                                          │
                                      airborne → descending → landing → settling → disarmed
```

- **ascending** issues a GUIDED **takeoff** (`MAV_CMD_NAV_TAKEOFF`) to each vehicle once it
  confirms armed, then waits for the autopilot to fly the climb to the hover altitude.
  Position setpoints are NOT sent during this phase — ArduCopter won't take off on
  setpoints alone and auto-disarms ~15 s after arming if no takeoff command arrives, so the
  setpoint-driven approach silently fails on real vehicles (and faithful SITL).
- **spreading / climbing / airborne / descending** are *formation* phases driven by guided
  position setpoints (midpoint + slots), with separation opened only once at hover and the
  midpoint altitude ramped between hover and the working altitude.
- **landing** switches each vehicle to **LAND** mode, which descends straight down and
  disarms on the ground. The net band and collision minimum are enforced in every phase
  that sends setpoints. `Engage now` bypasses the sequence for manual hover entry.

## Key decisions (the "why")

**Profiles, not magic auto-sizing.** An ArduPilot FC knows its frame *class* and tuning
but never its real span or mass — that data lives nowhere on the autopilot. So "read the
drone's size automatically" is physically impossible from the FC alone. Instead each
vehicle is matched to a `profile_id` and the real numbers come from config. This is the
honest version of auto-configuration, and it's what makes separation/collision maths
mean something.

**The net is a constraint, not decoration.** A net spanning two drones turns them into a
constrained pair: open them past the net width and it snaps taut and yanks them off
station; close them too far and, *if the cloth is on the same face as the props*, the
slack sags into the rotors. So linked pairs are held inside `[slack_min, span]`, and the
operator's separation is clamped to the net span *before any setpoint goes out*.

**Rig geometry is part of the physics.** Whether the props can foul the net depends on how
the aircraft is built, so each profile carries `inverted` (props-down, top deck free) and
`net_attach` (`top`/`bottom`). The safety layer derives *props-clear* from these: when the
net is on the opposite face from the props it can't reach them, so the slack-into-props
warning is suppressed — while the hard collision minimum still applies to every pair,
netted or not. The 3D model draws each airframe accordingly (rotors on the right side, a
free-deck ring when inverted) so what you see matches what the safety maths assumes. This,
plus the staged sequence, is the concrete answer to the propeller-entanglement risk.

**Staged take-off/landing over "arm and go."** Because the group is physically linked, the
controller never lifts or lands them as independent vehicles. The phase sequencer (above)
synchronises vertical motion, spreads only when airborne, and disarms only on the ground —
so the cloth is never yanked through a rotor disk during the riskiest moments.

**Reconfigure at runtime, rehearse before committing.** The dashboard opens on a Setup
screen and the controller can rebuild its link in place, so the intended workflow is
*configure → fly it in the simulator → switch the same setup to real radios*. The sim and
real paths are the same code above `VehicleLink`, so a sim rehearsal is a faithful dry run.

**One interface, many vehicles.** Everything above `VehicleLink` is written once. Adding
PX4-specific handling, a different radio, or a DDS bridge later means writing one new
link class — not editing the formation logic.

**Auto-config is runtime, provisioning is pre-flight.** The loop will put discovered
vehicles into GUIDED before commanding them (safe, reversible, runtime). Writing params
like `SYSID_THISMAV` or failsafe settings is a *provisioning* step done once on the
bench with a reboot — not something to do to a flying aircraft. Kept deliberately
separate.

---

## Bring-up checklist (SITL → real)

Do this in order. Don't skip to hardware.

**1. SITL, single vehicle.**
```bash
sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14550
python run.py --connect udp:127.0.0.1:14550
```
On the Setup screen, switch the source to *Real*, enter the connection string, set the rig,
and **Connect**. Confirm: the vehicle appears in the roster with the right autopilot/frame;
**Auto take-off** walks through ascending → spreading → climbing and hands you the midpoint;
**Hold** stops it; **Auto land** brings it down and disarms.

**2. SITL, multiple vehicles.** Launch several instances with distinct `SYSID_THISMAV` and
ports (`-I0 -I1 …`). Add each connection string on the Setup screen. Confirm the roster
shows them all with the right names/profiles, **Auto take-off** lifts them together and only
spreads once airborne, the line forms to the commanded spacing, and the net lines colour
correctly as you open separation past the net span. Check the 3D view matches the real
geometry (altitudes, heading, which face the props are on).

**3. Bench, real radios, props OFF.** Verify telemetry, GPS fix, mode changes, and that
**Disarm** and **Land** work instantly. Sanity-check that the geofence numbers in
`config.json` match your actual flying site.

**4. First flight, conservative.** Wide separation under the net span, low altitude,
small fence. Have a manual-takeover plan per vehicle (a pilot on the sticks who can flip
out of GUIDED). Engage, fly the midpoint gently, watch the safety bar.

Pre-flight every time: every vehicle has a **unique SYSID**; profiles match the real
airframes; fence matches the site; battery failsafes set on the FC (the GCS battery
checks are a backstop, not a substitute).

---

## Project status & next

- **Project A — generalise the framework:** done. Discovery, profiles (incl. rig
  geometry), swappable link with runtime reconfigure, N-drone formations, props-clear
  safety, staged take-off/landing, and a dashboard with a Setup flow plus 2D and 3D views.
  Tested in sim end-to-end (32 unit + 12 live checks).
- **Project B — close the autonomy loop:** not started. Trajectory prediction of the
  target → dynamic midpoint + separation commands so a capture runs itself. Drops on
  top of this loop as a module that writes the midpoint instead of the operator.

Smaller follow-ups: validate the MAVLink path in SITL; nearest-slot assignment to cut
crossings during re-spacing; a bench provisioning tool for SYSID/failsafe params.
