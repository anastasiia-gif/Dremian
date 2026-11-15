"""
Launch N *real* ArduCopter SITL instances and wire them to formation-gcs.

This uses the genuine ArduPilot SITL binary that Mission Planner already downloaded
onto your machine -- run directly, it opens TCP ports (unlike Mission Planner's own
Simulation button, which keeps them internal). Native Windows, no WSL, no Cygwin.

Each instance gets:
  * a unique MAVLink system id (1..N)  -> so formation-gcs sees them as separate drones
  * DISARM_DELAY = 0                   -> no 10 s ground auto-disarm while you set up
  * a home offset                      -> they don't spawn on top of each other
  * its own working dir                -> no eeprom/log clashes between instances
  * SERIAL0 on TCP 5760 + instance*10  -> drone 1 = 5760, drone 2 = 5770, drone 3 = 5780

Then formation-gcs connects to all of them over TCP and you fly the formation.

Usage:
  python -m tools.sitl_swarm --count 2
  python -m tools.sitl_swarm --count 3 --home 52.1454,4.5379,5,0
  python -m tools.sitl_swarm --count 2 --sitl "C:\\Users\\you\\Documents\\Mission Planner\\sitl\\ArduCopter.exe"
  python -m tools.sitl_swarm --count 2 --no-gcs   # just the drones; connect the GCS yourself

Want a MAVProxy console on a drone too? Each instance also exposes SERIAL1 on TCP
5762 + instance*10 (drone 1 -> 5762, drone 2 -> 5772). Point MAVProxy there while
formation-gcs uses 5760/5770:
  mavproxy --master=tcp:127.0.0.1:5762 --console
"""
from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys
import tempfile
import time

DEFAULT_HOME = (52.1454, 4.5379, 5.0, 0.0)   # Amsterdam-ish; override with --home


def find_sitl(explicit: str | None) -> str | None:
    if explicit:
        return explicit if os.path.isfile(explicit) else None
    roots = [
        os.path.expandvars(r"%USERPROFILE%\Documents\Mission Planner"),
        os.path.expandvars(r"%LOCALAPPDATA%\Mission Planner"),
        os.path.expandvars(r"%PROGRAMFILES(X86)%\Mission Planner"),
        os.path.expandvars(r"%PROGRAMFILES%\Mission Planner"),
    ]
    for root in roots:
        if not root or not os.path.isdir(root):
            continue
        for hit in glob.glob(os.path.join(root, "**", "*opter*.exe"), recursive=True):
            if "copter" in os.path.basename(hit).lower():
                return hit
    return None


def find_default_params(sitl_exe: str) -> str | None:
    base = os.path.dirname(sitl_exe)
    for up in (base, os.path.dirname(base)):
        hits = glob.glob(os.path.join(up, "**", "copter.parm"), recursive=True)
        if hits:
            return hits[0]
    return None


def write_param_file(base_params: str | None, sysid: int, workdir: str) -> str:
    overrides = {"SYSID_THISMAV": sysid, "DISARM_DELAY": 0}
    keep: list[str] = []
    if base_params and os.path.isfile(base_params):
        with open(base_params, "r", errors="ignore") as f:
            for line in f:
                parts = line.split()
                key = parts[0].upper().rstrip(",") if parts else ""
                if key in overrides:
                    continue
                keep.append(line.rstrip("\n"))
    keep.extend(f"{k} {v}" for k, v in overrides.items())
    path = os.path.join(workdir, f"copter{sysid}.parm")
    with open(path, "w") as f:
        f.write("\n".join(keep) + "\n")
    return path


def find_mavproxy(explicit: str | None) -> str | None:
    import shutil
    if explicit:
        return explicit if (os.path.isfile(explicit) or shutil.which(explicit)) else None
    for name in ("mavproxy", "mavproxy.exe", "mavproxy.py"):
        hit = shutil.which(name)
        if hit:
            return hit
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description="Launch real ArduCopter SITL instances for formation-gcs")
    ap.add_argument("--count", type=int, default=2, help="number of drones (default 2)")
    ap.add_argument("--sitl", default=None, help="path to ArduCopter.exe (auto-detected if omitted)")
    ap.add_argument("--home", default=None, help="lat,lon,alt,heading for drone 1")
    ap.add_argument("--speedup", type=int, default=1, help="sim speed multiplier (default 1 = real time)")
    ap.add_argument("--no-gcs", action="store_true", help="launch only the SITL drones, not formation-gcs")
    ap.add_argument("--mavproxy", action="store_true",
                    help="put a MAVProxy console in front of each drone (validate there), and have "
                         "formation-gcs ride MAVProxy's forwarded UDP instead of TCP")
    ap.add_argument("--mavproxy-path", default=None, help="path/name of the mavproxy executable")
    args = ap.parse_args()

    sitl = find_sitl(args.sitl)
    if not sitl:
        print("ERROR: couldn't find the ArduCopter SITL binary.\n")
        print("Fix: open Mission Planner -> SIMULATION -> Multirotor once (that downloads it),")
        print("then find ArduCopter.exe (search your 'Documents\\Mission Planner' folder) and pass it:")
        print('  python -m tools.sitl_swarm --count 2 --sitl "C:\\...\\Mission Planner\\sitl\\ArduCopter.exe"')
        return 1
    print("SITL binary :", sitl)

    home = DEFAULT_HOME
    if args.home:
        try:
            lat, lon, alt, hdg = (float(x) for x in args.home.split(","))
            home = (lat, lon, alt, hdg)
        except ValueError:
            print("ERROR: --home must be lat,lon,alt,heading e.g. 52.1454,4.5379,5,0")
            return 1

    base_params = find_default_params(sitl)
    print("params      :", base_params or "(SITL built-in defaults)")

    workroot = os.path.join(tempfile.gettempdir(), "formation_sitl")
    os.makedirs(workroot, exist_ok=True)

    mavproxy = None
    if args.mavproxy:
        mavproxy = find_mavproxy(args.mavproxy_path)
        if not mavproxy:
            print("\nNOTE: --mavproxy requested but couldn't find the mavproxy executable.")
            print("      I'll still start the drones; run MAVProxy yourself with the lines printed below.\n")

    new_console = subprocess.CREATE_NEW_CONSOLE if os.name == "nt" else 0
    conns: list[str] = []
    manual_mavproxy: list[str] = []
    print(f"\nlaunching {args.count} drone(s):")
    for i in range(args.count):
        sysid = i + 1
        tcp = 5760 + i * 10
        udp = 14550 + i * 10
        hlat = home[0] + i * 0.00003          # ~3 m apart so they don't overlap
        inst_dir = os.path.join(workroot, f"d{sysid}")
        os.makedirs(inst_dir, exist_ok=True)
        parm = write_param_file(base_params, sysid, inst_dir)
        cmd = [sitl, "-M", "quad", f"-I{i}", "-w",
               "--home", f"{hlat:.7f},{home[1]},{home[2]},{home[3]}",
               "--defaults", parm, "--speedup", str(args.speedup)]
        subprocess.Popen(cmd, cwd=inst_dir, creationflags=new_console)
        time.sleep(2)

        if args.mavproxy:
            mp_cmd = (f"--master=tcp:127.0.0.1:{tcp} --out=udp:127.0.0.1:{udp} "
                      f"--console --map")
            manual_mavproxy.append(f"mavproxy {mp_cmd}")
            if mavproxy:
                # MAVProxy talks to SITL and forwards to UDP; formation-gcs rides the UDP
                full = [mavproxy, f"--master=tcp:127.0.0.1:{tcp}",
                        f"--out=udp:127.0.0.1:{udp}", "--console", "--map"]
                subprocess.Popen(full, creationflags=new_console)
            conns.append(f"udpin:0.0.0.0:{udp}")
            print(f"  drone {sysid}: SITL tcp {tcp}  ->  MAVProxy  ->  udp {udp}  (GCS rides {udp})")
        else:
            conns.append(f"tcp:127.0.0.1:{tcp}")
            print(f"  drone {sysid}: TCP {tcp}  sysid {sysid}  home {hlat:.5f},{home[1]}")

    connect_line = "python run.py " + " ".join(f"--connect {c}" for c in conns)
    print("\nDrones are booting. Give them ~10 s to get GPS lock (watch their windows).")
    if args.mavproxy and not mavproxy:
        print("\nStart a MAVProxy for each drone (each gives you a console to validate):")
        for line in manual_mavproxy:
            print("  " + line)
        print("\nValidate in any MAVProxy console:  mode guided  ->  arm throttle  ->  takeoff 10")
    elif args.mavproxy:
        print("Each drone has its own MAVProxy window — validate there: "
              "mode guided -> arm throttle -> takeoff 10")
    print("\nformation-gcs connect line:\n  " + connect_line)

    if args.no_gcs:
        print("\n(--no-gcs) start the GCS yourself with the line above when the drones have GPS.")
        return 0

    print("\nwaiting 10 s for GPS lock, then starting formation-gcs ...")
    time.sleep(10)
    run_cmd = [sys.executable, "run.py"]
    for c in conns:
        run_cmd += ["--connect", c]
    return subprocess.call(run_cmd, cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


if __name__ == "__main__":
    raise SystemExit(main())
