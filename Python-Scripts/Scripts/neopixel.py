# Example usage of the new neopixel_control.py
from neopixel_control import NeoPixelController
import time
import cflib.crtp

# Initialize CRTP drivers
cflib.crtp.init_drivers()

# Create controller (update IP to match your drone)
neo = NeoPixelController("udp://192.168.43.42")
neo.connect()

try:
    # Set colors
    neo.set_pixel(0, 255, 255, 255)  # Red
    neo.set_pixel(1, 255, 255, 255)  # Green
    neo.set_pixel(2, 255, 255, 255)  # Blue
    neo.set_pixel(3, 255, 255, 255)  # White
    neo.show()

    # Start blinking
    neo.start_blink(500, 500)  # 500ms on/off
    time.sleep(4)

    # Stop blinking
    neo.stop_blink()

    # Clear all LEDs
    neo.clear()

finally:
    neo.disconnect()