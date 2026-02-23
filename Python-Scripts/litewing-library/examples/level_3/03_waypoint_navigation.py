"""
Level 3 — Waypoint Navigation
================================
Fly to absolute positions and follow multi-point paths.

What you'll learn:
    - fly_to(x, y) for absolute position navigation
    - fly_path() for multi-waypoint routes
    - Coordinate system: X = left/right, Y = forward/backward
    - Speed control during waypoint flight

Coordinate system (from drone's starting perspective):
    +Y = forward
    -Y = backward
    +X = left
    -X = right
"""

import time
from litewing import LiteWing

drone = LiteWing("192.168.43.42")
drone.target_height = 0.3

drone.connect()
time.sleep(2)

print(f"Battery: {drone.battery:.2f}V")

drone.start_logging("waypoint_flight.csv")

drone.arm()
drone.takeoff()

# ── Method 1: fly_to (absolute positions) ────────────
print("\n--- fly_to: Triangle pattern ---")

# Fly forward
drone.fly_to(0.0, 0.5, speed=0.3)
print(f"  Position: ({drone.position[0]:.2f}, {drone.position[1]:.2f})")

# Fly to the right
drone.fly_to(-0.5, 0.0, speed=0.3)
print(f"  Position: ({drone.position[0]:.2f}, {drone.position[1]:.2f})")

# Return to origin
drone.fly_to(0.0, 0.0, speed=0.3)
print(f"  Position: ({drone.position[0]:.2f}, {drone.position[1]:.2f})")

drone.hover(2)

# ── Method 2: fly_path (waypoint list) ───────────────
print("\n--- fly_path: Square pattern ---")

square = [
    (0.0, 0.3),   # Forward
    (-0.3, 0.3),  # Right-forward corner
    (-0.3, 0.0),  # Right
    (0.0, 0.0),   # Back to start
]

drone.fly_path(square, speed=0.3)
print(f"  Final: ({drone.position[0]:.2f}, {drone.position[1]:.2f})")

drone.land()
drone.stop_logging()
drone.disconnect()

print("\nDone! Check 'waypoint_flight.csv' for the full trajectory.")
