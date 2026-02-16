"""
LiteWing Example — Tier 1: Beginner
=====================================
Your first flight! Connect, take off, hover, and land.

What you'll learn:
    - How to connect to the drone
    - Arming as a safety concept
    - Basic flight commands
    - Checking battery voltage
"""

from litewing import LiteWing

# Create the drone controller
drone = LiteWing("192.168.43.42")

# Check battery before flying
sensors = drone.read_sensors()
print(f"Battery: {sensors.battery}V")

if sensors.battery > 0 and sensors.battery < 3.0:
    print("Battery too low! Charge before flying.")
else:
    # Arm and fly
    drone.arm()

    # hover_duration controls how long the drone stays in the air
    drone.fly(hover_duration=10)  # Hover for 10 seconds then land

    print("Flight complete!")
