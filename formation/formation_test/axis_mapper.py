#!/usr/bin/env python3
"""
axis_mapper.py — Quick utility to discover your TX16S joystick axis mapping.
Run this BEFORE formation_controller.py to verify stick axes and set
AXIS_ROLL, AXIS_PITCH, AXIS_THROTTLE, AXIS_YAW in the main script.

Usage:
    python axis_mapper.py
"""

import pygame
import time
import sys

def main():
    pygame.init()
    pygame.joystick.init()

    n = pygame.joystick.get_count()
    if n == 0:
        print("ERROR: No joystick found.")
        print("  1. Power on your TX16S")
        print("  2. Connect TOP USB-C port to laptop")
        print("  3. Select 'USB Joystick' on TX16S screen")
        sys.exit(1)

    print(f"Found {n} joystick(s):\n")
    for i in range(n):
        j = pygame.joystick.Joystick(i)
        j.init()
        print(f"  [{i}] {j.get_name()}")
    print()

    j = pygame.joystick.Joystick(0)
    j.init()
    num_axes    = j.get_numaxes()
    num_buttons = j.get_numbuttons()
    num_hats    = j.get_numhats()

    print(f"Using: {j.get_name()}")
    print(f"  Axes: {num_axes}  |  Buttons: {num_buttons}  |  Hats: {num_hats}")
    print()
    print("Move each stick/knob slowly and watch which axis changes.")
    print("Press Ctrl+C to exit.\n")

    header = "  " + "  ".join(f"Ax{i:02d}" for i in range(num_axes))
    print(header)
    print("  " + "-" * (6 * num_axes))

    try:
        while True:
            pygame.event.pump()
            axes = [j.get_axis(i) for i in range(num_axes)]
            # Highlight axes that are significantly deflected
            parts = []
            for v in axes:
                bar = f"{v:+.2f}"
                if abs(v) > 0.1:
                    bar = f"[{bar}]"
                else:
                    bar = f" {bar} "
                parts.append(bar)
            print(f"\r  " + " ".join(parts), end='', flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\nFinal mapping guide:")
        print("  Right stick X (Roll)     → Axis", end=' ')
        print("(deflect right stick left/right and note which axis moves)")
        print("  Right stick Y (Pitch)    → Axis (right stick up/down)")
        print("  Left  stick Y (Throttle) → Axis (left stick up/down)")
        print("  Left  stick X (Yaw)      → Axis (left stick left/right)")
        print("  VRA knob                 → Axis (turn the VRA knob)")
        print()
        print("Update these values at the top of formation_controller.py:")
        print("  AXIS_ROLL      = X")
        print("  AXIS_PITCH     = X")
        print("  AXIS_THROTTLE  = X")
        print("  AXIS_YAW       = X")
        print("  AXIS_SEP_KNOB  = X")
        pygame.quit()

if __name__ == '__main__':
    main()
