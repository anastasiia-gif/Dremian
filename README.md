# Dremian — Graduation Internship: Multi-Drone Formation Flight

Work from my graduation internship: flying **N drones as one rigid formation** for a net-capture rig (a cloth strung between drones), from simulation to real MAVLink vehicles.

## Repository layout

| Folder | Contents |
|---|---|
| [`formation-gcs/`](formation-gcs/formation-gcs/) | **The main project** — a ground control station that auto-discovers vehicles, attaches physical profiles and flies the whole group as one formation (translate, climb, turn, re-space) over a simulator or real MAVLink with no code changes. Includes tests, tools and [architecture docs](formation-gcs/formation-gcs/docs/ARCHITECTURE.md). |
| [`formation/`](formation/) | Earlier formation experiments and flight-test material, including the [formation guide](formation/formation_test/formation_guide.md) and home-testing setup. |

## Where to start

Read [`formation-gcs/formation-gcs/README.md`](formation-gcs/formation-gcs/README.md) — it covers the Setup → Simulate → Connect → Fly workflow, the safety layers and the current project status.
