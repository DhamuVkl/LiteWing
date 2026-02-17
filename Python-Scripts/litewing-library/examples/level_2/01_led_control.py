"""
Level 2 — LED Control
======================
Control the NeoPixel LEDs on the drone.

What you'll learn:
    - Setting solid LED colors (RGB)
    - Making LEDs blink
    - Turning LEDs off

No motors are started — safe to run on your desk!
"""

import time
from litewing import LiteWing

drone = LiteWing("192.168.43.42")
drone.connect()

# ── Solid colors ─────────────────────────────────────

print("Red...")
drone.set_led_color(255, 0, 0)
time.sleep(1)

print("Green...")
drone.set_led_color(0, 255, 0)
time.sleep(1)

print("Blue...")
drone.set_led_color(0, 0, 255)
time.sleep(1)

print("White...")
drone.set_led_color(255, 255, 255)
time.sleep(1)

print("Purple...")
drone.set_led_color(128, 0, 255)
time.sleep(1)

# ── Blinking ─────────────────────────────────────────

print("Blinking (fast)...")
drone.set_led_color(128, 0, 255)  # Orange
drone.blink_leds(on_ms=200, off_ms=200)
time.sleep(3)

print("Blinking (slow)...")
drone.blink_leds(on_ms=800, off_ms=800)
time.sleep(3)

# ── Turn off ─────────────────────────────────────────

print("LEDs off.")
drone.clear_leds()
time.sleep(1)

# Disconnect
drone.disconnect()
print("Done!")
