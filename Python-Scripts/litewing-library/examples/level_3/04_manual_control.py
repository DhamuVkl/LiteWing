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

Note: start_manual_control() handles connect, arm, takeoff, and landing
internally. You do NOT need to call connect() or arm() before this.
"""

from litewing import LiteWing

drone = LiteWing("192.168.43.42")
drone.target_height = 0.3

# ── Manual control settings ──────────────────────────

# How fast the drone responds to key input
drone.sensitivity = 0.2  # Default: 0.2 (m/s per key)

# What happens when you release all keys?
drone.hold_mode = "current"  # "current" = stay here, "origin" = go back

print("Manual Control Settings:")
print(f"  Sensitivity: {drone.sensitivity}")
print(f"  Hold mode:   {drone.hold_mode}")
print()
print("Controls: WASD to move, SPACE or Q to land")
print()

# ── Start manual control ─────────────────────────────
# This starts a background thread that handles:
#   connect → arm → takeoff → WASD loop → land → disconnect
drone.start_manual_control()

# Wait for manual control to finish (user presses SPACE/Q to land)
if drone._manual_thread:
    drone._manual_thread.join()

print("Manual control ended.")
print("Done!")
