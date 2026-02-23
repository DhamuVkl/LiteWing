"""
Level 2 — Tuning & Configuration
==================================
Learn how to adjust drone settings before flight.

What you'll learn:
    - Setting target height
    - Adjusting thrust base (motor power)
    - Changing hover duration
    - Tuning PID gains for position hold
    - Applying firmware parameters

These settings affect how the drone flies. Experiment with
small changes to see the difference!
"""

import time
from litewing import LiteWing

drone = LiteWing("192.168.43.42")

# ─── Flight settings ────────────────────────────────
# These control the basic flight behavior:

drone.target_height = 0.4      # Hover at 40cm (default: 0.3m)
drone.hover_duration = 1.0     # Hover for 5 seconds
drone.maneuver_distance = 0.3  # Default distance for forward/left/etc.

# ─── Motor power ────────────────────────────────────
# thrust_base is the base motor power (PWM value).
# Higher = more thrust. Adjust if the drone is too heavy or too light.

drone.thrust_base = 24000      # Default: 24000 (range: ~20000–30000)

# ─── Height controller ──────────────────────────────
# These control how the firmware maintains height:

drone.z_position_kp = 2.0     # Position gain: how hard it tries to reach target height
drone.z_velocity_kp = 25.0    # Velocity gain: how much it dampens vertical speed

# ─── Position hold PID ──────────────────────────────
# These control how the drone holds its XY position
# using optical flow feedback.

# Position PID: controls drift correction
drone.position_pid.kp = 1.0    # Proportional: react to position error
drone.position_pid.ki = 0.03   # Integral: correct steady-state drift
drone.position_pid.kd = 0.0    # Derivative: dampen oscillations

# Velocity PID: controls speed response
drone.velocity_pid.kp = 0.7
drone.velocity_pid.ki = 0.01
drone.velocity_pid.kd = 0.0

# ─── Print current config ──────────────────────────
print("Current Configuration:")
print(f"  Target height:    {drone.target_height} m")
print(f"  Hover duration:   {drone.hover_duration} s")
print(f"  Thrust base:      {drone.thrust_base}")
print(f"  Z position Kp:    {drone.z_position_kp}")
print(f"  Z velocity Kp:    {drone.z_velocity_kp}")
print(f"  Position PID:     {drone.position_pid}")
print(f"  Velocity PID:     {drone.velocity_pid}")
print()

# ─── Test flight with tuned settings ────────────────
drone.connect()
time.sleep(2)

print(f"Battery: {drone.battery:.2f}V")

# Apply firmware parameters (thrust, height gains)
drone.apply_firmware_params()
print("Firmware parameters applied.")

# Fly with tuned config
drone.arm()
drone.takeoff()
drone.hover(drone.hover_duration)
drone.land()

drone.disconnect()
print("Done! Adjust values and re-run to compare.")
