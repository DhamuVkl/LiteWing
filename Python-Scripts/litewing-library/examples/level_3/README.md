# Level 3 — Advanced Control

Deep-dive into the drone's control systems. These examples build on
Level 2 flight skills.

| # | Script | What it teaches |
|---|--------|-----------------|
| 01 | `01_pid_tuning.py` | Position & velocity PID gains, effects of tuning |
| 02 | `02_position_hold.py` | Position hold config, enable/disable mid-flight |
| 03 | `03_waypoint_navigation.py` | `fly_to()` and `fly_path()` for absolute position nav |
| 04 | `04_manual_control.py` | WASD keyboard control with hold modes |
| 05 | `05_firmware_params.py` | Thrust, height PID on the onboard controller |
| 06 | `06_led_flight_effects.py` | LED status indicator during flight phases |

## Prerequisites

- Complete Level 2 examples first
- Comfortable with arm → takeoff → land sequence
- Know where `emergency_stop()` is!
