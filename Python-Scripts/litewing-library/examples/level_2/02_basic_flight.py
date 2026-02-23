"""
Level 2 — Basic Flight
========================
The simplest flight: arm, take off, hover, and land.

What you'll learn:
    - The arm → takeoff → land sequence
    - What target_height controls
    - How to safely test your first flight

⚠️ SAFETY FIRST:
    - Clear the area around the drone
    - Stay at least 1 meter away
    - Be ready to run emergency_stop() if needed
"""

import time
from litewing import LiteWing

drone = LiteWing("192.168.43.42")

# ── Configure before flight ──────────────────────────
drone.target_height = 0.3    # Hover at 30cm (keep it low for first tests!)
drone.hover_duration = 3.0   # Hover for 3 seconds

# ── Connect ──────────────────────────────────────────
drone.connect()
time.sleep(2)

# Pre-flight check
print(f"Battery: {drone.battery:.2f}V")
if drone.battery < 3.3:
    print("Battery too low! Charge before flying.")
    drone.disconnect()
    exit()

# ── LED indicator: ready to fly ──────────────────────
drone.set_led_color(0, 255, 0)  # Green = ready
time.sleep(1)

# ── Fly! ─────────────────────────────────────────────
print("Arming...")
drone.arm()

print(f"Taking off to {drone.target_height}m...")
drone.takeoff()

print(f"Hovering for {drone.hover_duration}s...")
drone.hover(drone.hover_duration)

print("Landing...")
drone.land()

# ── Done ─────────────────────────────────────────────
drone.clear_leds()
drone.disconnect()
print("Flight complete!")
