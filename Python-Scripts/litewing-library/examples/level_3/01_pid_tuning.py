"""
Level 3 — PID Tuning
======================
Understand and tune the PID controllers that keep the drone stable.

What you'll learn:
    - What position_pid and velocity_pid do
    - How to change gains (kp, ki, kd)
    - Observing the effect of different settings

The drone uses a CASCADED PID controller:
    1. Position PID → computes a velocity setpoint from position error
    2. Velocity PID → dampens the velocity to prevent overshoot

Tuning flow:
    - Start with default values
    - Increase position Kp for faster correction (but more oscillation)
    - Increase velocity Kp for more damping (smoother but slower)
    - Add small Ki to correct persistent drift
"""

import time
from litewing import LiteWing

drone = LiteWing("192.168.43.42")
drone.target_height = 0.3

# ── Default PID values (try this first) ──────────────
# Position PID: reacts to how far the drone is from the target
drone.position_pid.kp = 1.0    # Proportional: push toward target
drone.position_pid.ki = 0.03   # Integral: fix steady-state drift
drone.position_pid.kd = 0.0    # Derivative: dampen oscillation

# Velocity PID: reacts to how fast the drone is moving
drone.velocity_pid.kp = 0.7    # Proportional: resist unwanted speed
drone.velocity_pid.ki = 0.01   # Integral: correct persistent velocity error
drone.velocity_pid.kd = 0.0    # Derivative: smooth rapid changes

print("Current PID Settings:")
print(f"  Position: {drone.position_pid}")
print(f"  Velocity: {drone.velocity_pid}")
print()

# ── Connect and fly ──────────────────────────────────
drone.connect()
time.sleep(2)

print(f"Battery: {drone.battery:.2f}V")

drone.start_logging("pid_test.csv")

drone.arm()
drone.takeoff()

# Hover for 5 seconds — watch how stable it is
print("Hovering with current PID settings...")
drone.wait(5)

# Move forward and back — watch the corrections
print("Testing movement response...")
drone.forward(0.3)
drone.wait(2)
drone.backward(0.3)
drone.wait(2)

drone.land()
drone.stop_logging()
drone.disconnect()

print("\nDone! Check 'pid_test.csv' to see the corrections.")
print()
print("Experiment with these changes:")
print("  - Higher position Kp (1.5) → more aggressive correction")
print("  - Lower position Kp (0.5) → gentler, allows more drift")
print("  - Higher velocity Kp (1.0) → more damping, less overshoot")
print("  - Add position Kd (0.05)  → reduces oscillation")
