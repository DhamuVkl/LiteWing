"""
Level 1 — Live Position Trail (GUI)
=====================================
Open a live 2D map showing the drone's movement trail.
Uses dead reckoning from the PMW3901 optical flow sensor.

Move the drone around on a flat surface to see the
XY position trail draw in real time!

  - Green dot = current position
  - Red square = starting position
  - Blue trail = movement history

Close the window or press Ctrl+C to stop.
"""

from litewing import LiteWing
from litewing.gui import live_position_plot

drone = LiteWing("192.168.43.42")
live_position_plot(drone)
