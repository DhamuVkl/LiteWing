"""
Hello LiteWing! — Your first flight using the litewing library.
Based on the original hellow_litewing.py pattern.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "Flight_Stabilizer_Shield"))

from litewing import LiteWing
import time

# Create drone controller
drone = LiteWing("192.168.43.42")
drone.target_height = 0.3
drone.hover_duration = 10

print("Hello World to LiteWing!")

def my_flight(drone_ref, cf, has_pos_hold):
    """Runs once the drone is airborne."""
    from litewing._flight_engine import _hover_loop

    # Read sensors while connected — battery works here!
    sensors = drone_ref.read_sensors()
    print(f"Battery: {sensors.battery:.2f}V")
    print(f"Height:  {sensors.height:.3f}m")

    if sensors.battery > 0 and sensors.battery < 3.0:
        print("WARNING: Battery low!")

    # Hover with position hold
    _hover_loop(drone_ref, cf, has_pos_hold, drone_ref.hover_duration)

drone.arm()
drone.fly(maneuver_fn=my_flight)
print("Test complete")