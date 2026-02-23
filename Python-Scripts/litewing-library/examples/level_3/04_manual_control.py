"""
Level 3 — Manual Control
==========================
Fly the drone with WASD keyboard controls.

What you'll learn:
    - Starting manual (keyboard) control
    - Hold modes: "current" vs "origin"
    - Sensitivity and momentum settings
    - Stopping manual control safely

Controls:
    W = Forward    S = Backward
    A = Left       D = Right
    SPACE or Q = Land and stop

Hold modes:
    "current" — release keys → hold at current position (free flight)
    "origin"  — release keys → snap back to launch point (spring mode)
"""

import time
from litewing import LiteWing

drone = LiteWing("192.168.43.42")
drone.target_height = 0.3

# ── Manual control settings ──────────────────────────

# How fast the drone responds to key input
drone.sensitivity = 0.2  # Default: 0.2 (m/s per key)

# What happens when you release all keys?
drone.hold_mode = "current"  # "current" = stay here, "origin" = go back

# How long the drone takes to stop after releasing keys
drone.momentum_compensation_time = 0.3  # seconds

print("Manual Control Settings:")
print(f"  Sensitivity: {drone.sensitivity}")
print(f"  Hold mode:   {drone.hold_mode}")
print()
print("Controls: WASD to move, SPACE or Q to land")
print()

# ── Start flying ─────────────────────────────────────
drone.connect()
time.sleep(2)

print(f"Battery: {drone.battery:.2f}V")

# LED indicator
drone.set_led_color(0, 255, 0)  # Green = manual mode active

# This blocks until you press SPACE/Q or call stop_manual_control()
drone.start_manual_control()

print("Manual control ended.")
drone.clear_leds()
drone.disconnect()
print("Done!")
