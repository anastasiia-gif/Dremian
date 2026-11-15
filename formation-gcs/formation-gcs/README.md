# formation-gcs

Ground station for flying **N drones as one rigid formation** — a virtual "stick" with
a midpoint you pilot, and the whole group translates, climbs, turns and re-spaces
together. Built for the net-capture rig (cloth strung between drones), but the
formation/safety/UI layers don't care what's hanging off the airframes.

It auto-discovers whatever vehicles show up, identifies them, attaches a physical
profile, and runs the same control loop over a **simulator** or **real MAVLink** with no
code changes.

> **Status.** Project A (generalise the framework) is in and tested. Project B (close the
> autonomy loop — trajectory prediction driving the formation) is the next phase. See
> [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

---

## The flow: Setup → Test in sim → Connect → Fly

The app opens on a **Setup screen**, not a live cockpit. Nothing is commanded until you
launch. The intended path:

1. **Configure** the fleet, the catching rig, the formation and the geofence.
2. **Launch in simulator** and rehearse the whole mission — take-off, formation, landing —
   with zero risk.
3. **Connect to drones** (same screen, switch the source to *Real*) once you're happy.
4. **Fly**: telemetry populates the roster, you place the formation and pilot the midpoint.

You can jump back to **Setup** from the cockpit header at any time to re-configure; the
backend rebuilds the link on the fly (sim ↔ real, fleet size, profiles, fence) without a
restart.

## Quick start (simulator — no hardware)

```bash
pip install -r requirements.txt        # only aiohttp is needed for sim
python run.py                          # opens the browser for you
```

`run.py` serves the dashboard and opens it automatically (use `--no-browser` to suppress).
On the **Setup** screen: leave the source on *Simulator*, pick a drone count, choose the
rig, then **Launch in simulator**. In the cockpit:

1. **Auto take-off** (set a working altitude). The group lifts through a staged sequence
   (see below) and hands you the midpoint when it's airborne.
2. Fly the midpoint with the keyboard or on-screen pad; pick a shape and drag
   **Separation**. Net lines turn amber/red as you approach the cloth's span.
3. **Auto land** brings everyone down and disarms on the ground.

The simulator models each drone as a point mass honouring its profile's speed and
acceleration limits, so formation behaviour, the safety bands, the staged sequence and
both views all behave like the real thing.

---

## The catching rig (prop & net geometry)

A net strung between drones can foul the props. The rig toggle on the Setup screen (and
each profile) captures how the aircraft are built so the **safety logic and the 3D model
match reality**:

- **Net below props (normal).** Conventional multirotor — props on top, cloth hung
  underneath. The slack net can't reach the props.
- **Inverted · top free.** Drone flown **props-down** with the **top deck left free** for
  the net. Props sit on the opposite face, so a sagging net stays clear of them.

Both are *props-clear* configurations. When the net is on the **same** face as the props,
the separation check adds a **slack-into-props** warning if the drones drift close enough
to let the cloth bag into the rotors; when the rig is props-clear, that warning is
suppressed (it can't happen) while the hard **collision** minimum still applies to every
pair — netted or not. The roster cards show each drone's rig and whether its props are
clear.

---

## Staged take-off, landing & disarm

Lifting a netted group all at once lets one drone snatch the cloth into another's props.
**Auto take-off** and **Auto land** run a sequence that avoids that:

```
disarmed → arming → ascending → spreading → climbing → AIRBORNE
                                                          │
                                      airborne → descending → landing → settling → disarmed
```

- **ascending** — each vehicle does a GUIDED **takeoff** to a low hover. (ArduCopter won't
  climb on position setpoints alone — it needs an explicit takeoff command, and it
  auto-disarms ~15 s after arming if none arrives. The sequence issues the takeoff right
  after arming.) The net lifts as the group reaches hover.
- **spreading** — only once at hover does the group open from tight ground spacing to your
  commanded separation.
- **climbing** — rise to the working altitude, then you get the midpoint.
- **descending → landing** — reverse on the way down: back to hover, then **LAND** mode
  brings each vehicle straight down and disarms it on the ground.

The net band and collision minimum are enforced the whole way through. The current phase
is shown as a pill in the header. `Hold` freezes the midpoint; `Engage now` is a manual
shortcut that jumps straight to formation flight at the current position (for when you're
already hovering).

---

## The scope: 2D plan and 3D view

Toggle in the top-right of the scope:

- **Plan (2D)** — top-down tactical view: range rings, geofence, net tension, trails,
  heading chevrons, midpoint reticle. Best for precise spacing.
- **3D** — a perspective view of the actual formation in space: ground grid, geofence,
  per-drone altitude lines and shadows, the sagging net on its attach face, and a stylised
  airframe that draws its rotors on the correct side (and a violet free-deck ring when
  inverted). **Drag to orbit, scroll to zoom**; the camera follows the midpoint. Use it to
  check that the simulated motion matches what the real drones do.

The 3D view is a self-contained canvas renderer — no external libraries, works offline.

## Operator controls

| Key            | Action                          | Key      | Action                |
|----------------|---------------------------------|----------|-----------------------|
| `W` / `S`      | midpoint forward / back         | `Q` / `E`| yaw left / right      |
| `A` / `D`      | strafe left / right             | `[` / `]`| separation − / +      |
| `R` / `F`      | climb / descend                 | `space`  | **hold**              |

Forward is always *along the formation's heading*, so it flies like one aircraft. On a
tablet, use the on-screen pad.

---

## Running against real drones

Works with ArduPilot or PX4 over any pymavlink connection string. One connection can carry
several vehicles (distinct `SYSID`), or use one connection per vehicle. Either start from
the Setup screen (*Real* source, paste connection strings, **Connect to drones**), or boot
straight into a real link:

```bash
pip install -r requirements.txt        # includes pymavlink

python run.py \
  --connect udp:0.0.0.0:14550 \
  --connect udp:0.0.0.0:14560
```

Map each `SYSID` to a name and a profile on the Setup screen or in `config.json`. The
MAVLink link path needs SITL or real radios to exercise — the simulator is the unit-tested
one. **Read the bring-up checklist in [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) before
flying.**

### Real ArduCopter SITL — multiple drones, native Windows

Mission Planner's built-in *Simulation* runs the real SITL binary but keeps its ports
internal, so nothing else can attach. Run that same binary yourself and it opens TCP ports
just like `sim_vehicle.py` — no WSL, no Cygwin. `tools/sitl_swarm.py` launches N instances
(unique `SYSID_THISMAV` per drone, `DISARM_DELAY=0`, separated home + working dirs, SERIAL0
on `5760 + instance*10`) and connects the GCS to all of them:

```bash
# Mission Planner -> SIMULATION -> Multirotor once, to download the SITL binary, then:
python -m tools.sitl_swarm --count 2
# or point it explicitly:
python -m tools.sitl_swarm --count 3 --sitl "C:\Users\you\Documents\Mission Planner\sitl\ArduCopter.exe"
```

Drone 1 is on `tcp:127.0.0.1:5760`, drone 2 on `5770`, etc.; the GCS is launched wired to
all of them. Want a MAVProxy console on a drone as well? Each instance also exposes SERIAL1
on `5762 + instance*10`, so `mavproxy --master=tcp:127.0.0.1:5762 --console` rides alongside
the GCS on drone 1.

**ArduCopter takeoff rule (the thing that bites everyone):** a copter on the ground in
GUIDED will *not* climb toward a position setpoint — it needs an explicit takeoff command
(`MAV_CMD_NAV_TAKEOFF`) while armed, and it auto-disarms ~15 s after arming if no takeoff
starts. The GCS **Auto take-off** issues that command for you; the launcher sets
`DISARM_DELAY=0` so the arm→setup→takeoff window isn't a race. If you drive a drone by hand
in MAVProxy instead: `mode guided` → `arm throttle` → `takeoff 10`.

---

## Configuration (`config.json`)

The Setup screen writes the same fields live, but a file is handy for fixed rigs:

```jsonc
{
  "use_sim": true,
  "sim_count": 3,
  "connections": [],              // real conn strings; overrides sim if non-empty
  "ws_host": "127.0.0.1",
  "ws_port": 8765,
  "loop_hz": 10.0,

  "origin": { "lat": null, "lon": null, "alt": null },  // null -> first GPS fix

  "names":          { "1": "Alpha", "2": "Bravo", "3": "Charlie" },
  "sysid_profiles": { "1": "8inch", "2": "8inch", "3": "10inch" },

  "profiles": [                   // extend the built-in library
    { "profile_id": "net8", "name": "8\" net carrier",
      "span_m": 0.42, "mass_kg": 1.3, "max_speed_ms": 10, "max_accel_ms2": 6,
      "inverted": true, "net_attach": "top" }    // props-down, cloth on the free top deck
  ],

  "fence": { "enabled": true, "center_e": 0, "center_n": 0,
             "radius_m": 80, "alt_min_m": 2, "alt_max_m": 35 },

  "sequence": {                   // staged take-off / landing
    "hover_alt_m": 2.5, "work_alt_m": 10.0,
    "ascent_rate_ms": 0.6, "climb_rate_ms": 1.0, "descent_rate_ms": 0.5,
    "settle_s": 1.5, "start_separation_m": 1.5 }
}
```

A **profile** is the stuff the flight controller does *not* know — real rotor span, mass,
performance envelope, and now the **rig geometry** (`inverted`, `net_attach`). The FC knows
it's a quad; it does not know it's 0.42 m across or flown upside-down. That gap is why
profiles exist. Built-in profiles: `default`, `5inch`, `8inch`, `10inch`.

Env overrides for the field: `FORMATION_CONNS`, `FORMATION_WS_HOST`, `FORMATION_WS_PORT`,
`FORMATION_SIM=1`.

---

## Project layout

```
run.py                      entry point (--sim / --connect / --port / --no-browser)
config.json                 fleet + fence + profiles + sequence
backend/
  models.py                 dataclasses: state, profile (+ rig geometry), target, spec
  config.py                 config load + built-in profiles + sequence params
  discovery.py              fleet roster: identify, profile, track join/leave   <- Project A
  protocol.py               websocket JSON in/out (config, state, roster, events)
  app.py                    aiohttp server + 10 Hz loop + flight-phase sequencer + reconfigure
  formation/
    geometry.py             midpoint -> per-drone targets, ENU <-> GPS
    safety.py               separation, net band (props-clear aware), fence, health, clamps
  sources/
    base.py                 VehicleLink interface
    sim_source.py           SimLink   (tested, hardware-free)
    mavlink_source.py       MAVLinkLink (real; needs SITL/hardware to validate)
frontend/index.html         single-file dashboard: Setup screen + cockpit (2D plan + 3D view)
tools/
  fake_autopilot.py         fake ArduCopter MAVLink endpoint (test/demo fixture)
  fake_swarm.py             live: fake MAVLink swarm + dashboard (python -m tools.fake_swarm)
tests/
  test_core.py              geometry / safety / rig / sim       (python -m tests.test_core)
  smoke_test.py             live server: sequencer, flying, reconfigure
  mavlink_loopback.py       real MAVLink over UDP via fake autopilots (needs pymavlink)
  position_crosscheck.py    station positions vs pyproj geodesy   (needs pyproj)
docs/ARCHITECTURE.md        design, decisions, SITL bring-up, what's next
```

## Testing

Four layers, fastest to most independent:

```bash
python -m tests.test_core           # 34 checks: geometry, safety, rig geometry, sim convergence
python -m tests.smoke_test          # 12 checks: staged take-off/land, midpoint flight, reconfigure
python -m tests.mavlink_loopback    # 16 checks: REAL MAVLink link incl. GUIDED takeoff (pymavlink)
python -m tests.position_crosscheck #  3 checks: station positions vs pyproj geodesy (unbiased)
```

`mavlink_loopback` stands up fake ArduCopter endpoints that speak genuine MAVLink and
drives a full staged take-off/landing through the real `MAVLinkLink` — validating discovery,
identification, telemetry decode, mode/arm commands, guided setpoints and the closed loop
with no autopilot or SITL.

`position_crosscheck` is the **bias check**: it takes the positions the station reports and
re-derives them with pyproj (the PROJ geodesy engine used by QGIS/GDAL — a completely
separate implementation) and confirms they agree to centimetres. To *watch* the MAVLink path
drive the dashboard live:

```bash
python -m tools.fake_swarm --count 3      # fake swarm + dashboard; press Auto take-off
```

For real flight dynamics (EKF, mode logic, true physics) before hardware, point
`run.py --connect` at ArduPilot SITL — see the bring-up checklist in `docs/ARCHITECTURE.md`.

## Known limits / what's next

- **MAVLink path is unflown here** — written to spec, validate in SITL first.
- **Slot assignment is by sysid** (deterministic), not yet nearest-slot optimal.
- **Param provisioning** (SYSID, failsafes) is a *pre-flight* step, deliberately not done
  mid-air. A small provisioning tool is a sensible next addition.
- **Per-drone rig editing** in the Setup UI is fleet-wide for now (one rig toggle); mixed
  rigs are still expressible per-profile in `config.json`.
- **Project B**: trajectory prediction → dynamic re-spacing for automated capture.
