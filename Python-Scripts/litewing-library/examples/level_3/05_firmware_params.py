"""
Level 3 — Firmware Parameters
================================
Tune the drone's onboard flight controller.

What you'll learn:
    - thrust_base: base motor power
    - z_position_kp: height position gain
    - z_velocity_kp: vertical velocity damping
    - Applying parameters to the firmware

These parameters are sent to the ESP32 flight controller
and affect how the drone maintains height.
"""

import time
from litewing import LiteWing

drone = LiteWing("192.168.43.42")
drone.target_height = 0.3

# ── Firmware parameters ──────────────────────────────

# thrust_base: base PWM thrust value
# Higher = more power to motors. Adjust based on drone weight/battery.
# Too low → drone can't take off. Too high → overshoots height.
drone.thrust_base = 24000      # Default: 24000 (range: ~20000-30000)

# z_position_kp: how aggressively the firmware corrects height error
# Higher = faster height correction, but may oscillate
drone.z_position_kp = 2.0      # Default: 2.0

# z_velocity_kp: how much the firmware dampens vertical speed
# Higher = smoother height changes, but slower response
drone.z_velocity_kp = 25.0     # Default: 25.0

print("Firmware Parameters:")
print(f"  thrust_base:    {drone.thrust_base}")
print(f"  z_position_kp:  {drone.z_position_kp}")
print(f"  z_velocity_kp:  {drone.z_velocity_kp}")
print()

# ── Connect and apply ────────────────────────────────
drone.connect()
time.sleep(2)

print(f"Battery: {drone.battery:.2f}V")

# Apply parameters to the firmware
drone.apply_firmware_params()
print("Firmware parameters applied!")

# Fly to test the settings
drone.arm()
drone.takeoff()
drone.wait(5)
drone.land()

drone.disconnect()

print("\nDone!")
print("Experiment with these changes:")
print("  thrust_base 22000 → less thrust (lighter drone / full battery)")
print("  thrust_base 26000 → more thrust (heavier drone / low battery)")
print("  z_position_kp 3.0 → faster height correction")
print("  z_velocity_kp 30.0 → smoother height transitions")
