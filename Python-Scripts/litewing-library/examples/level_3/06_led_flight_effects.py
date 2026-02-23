"""
Level 3 — LED Flight Effects
===============================
Change LED colors during flight to show drone state.

What you'll learn:
    - Using LEDs as flight status indicators
    - Changing colors during different flight phases
    - Blinking to signal warnings or events

Flight status colors:
    Green  = Ready / Hovering
    Blue   = Moving
    Orange = Warning / Low battery
    Red    = Landing
    Off    = Idle
"""

import time
from litewing import LiteWing

drone = LiteWing("192.168.43.42")
drone.target_height = 0.3

drone.connect()
time.sleep(2)

print(f"Battery: {drone.battery:.2f}V")

# ── Pre-flight: battery check with LED feedback ──────
if drone.battery < 3.5:
    print("Low battery!")
    drone.set_led_color(255, 165, 0)  # Orange warning
    drone.blink_leds(on_ms=300, off_ms=300)
    time.sleep(3)
    drone.clear_leds()
    drone.disconnect()
    exit()

# ── Green = ready ────────────────────────────────────
print("Status: READY")
drone.set_led_color(0, 255, 0)
time.sleep(1)

# ── Takeoff ──────────────────────────────────────────
drone.arm()
drone.takeoff()

# ── Green = hovering ─────────────────────────────────
print("Status: HOVERING")
drone.set_led_color(0, 255, 0)
drone.wait(2)

# ── Blue = moving ────────────────────────────────────
print("Status: MOVING")
drone.set_led_color(0, 100, 255)
drone.forward(0.3)
drone.wait(1)

drone.set_led_color(0, 100, 255)
drone.backward(0.3)
drone.wait(1)

# ── Green = hovering again ───────────────────────────
print("Status: HOVERING")
drone.set_led_color(0, 255, 0)
drone.wait(2)

# ── Red = landing ────────────────────────────────────
print("Status: LANDING")
drone.set_led_color(255, 0, 0)
drone.land()

# ── Off = idle ───────────────────────────────────────
drone.clear_leds()
drone.disconnect()
print("Done!")
