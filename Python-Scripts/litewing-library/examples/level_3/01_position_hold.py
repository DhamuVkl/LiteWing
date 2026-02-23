"""
Level 3 — Position Hold Configuration
========================================
Fine-tune how the drone holds its position.

What you'll learn:
    - max_correction: speed limit for corrections
    - optical_flow_scale: sensor calibration factor
    - enable/disable position hold mid-flight
    - Drift compensation settings
"""

import time
from litewing import LiteWing

drone = LiteWing("192.168.43.42")
drone.target_height = 0.3

# ── Position hold parameters ─────────────────────────

# max_correction: caps the PID output velocity (m/s)
# Higher = more aggressive corrections, lower = gentler
drone.max_correction = 0.7  # Default: 0.7

# optical_flow_scale: calibration for the PMW3901 sensor
# Adjust if the drone consistently overshoots or undershoots distances
drone.optical_flow_scale = 4.4  # Default: 4.4

# use_height_scaling: adjust optical flow reading based on height
# Higher altitude = sensor sees more ground movement per pixel
drone.use_height_scaling = True  # Default: True

# velocity_smoothing: how much to smooth velocity estimates (0-1)
# Higher = smoother but more latency
drone.velocity_smoothing = 0.3  # Default: 0.3

print("Position Hold Config:")
print(f"  max_correction:    {drone.max_correction}")
print(f"  optical_flow_scale: {drone.optical_flow_scale}")
print(f"  use_height_scaling: {drone.use_height_scaling}")
print()

# ── Connect and fly ──────────────────────────────────
drone.connect()
time.sleep(2)

print(f"Battery: {drone.battery:.2f}V")

drone.arm()
drone.takeoff()

# Phase 1: Normal position hold — GREEN steady
print("Phase 1: Position hold ON — watch stability")
drone.set_led_color(0, 255, 0)
drone.wait(5)

# Phase 2: Disable position hold — RED fast blink (warning!)
print("Phase 2: Position hold OFF — observe drift")
drone.set_led_color(255, 0, 0)
drone.blink_leds(on_ms=150, off_ms=150)
drone.disable_position_hold()
drone.wait(1.5)

# Phase 3: Re-enable position hold — BLUE steady (recovered)
print("Phase 3: Position hold ON — drone corrects")
drone.enable_position_hold()
drone.set_led_color(0, 100, 255)
drone.wait(3)

# Landing — clear LEDs
drone.clear_leds()
drone.land()
drone.disconnect()

print("\nDone!")
print("Try changing max_correction to see the effect:")
print("  0.3 → gentle corrections (may drift)")
print("  0.7 → balanced (default)")
print("  1.0 → aggressive corrections (may oscillate)")
