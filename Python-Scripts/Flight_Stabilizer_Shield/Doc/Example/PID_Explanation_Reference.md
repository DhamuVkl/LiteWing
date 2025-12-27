# PID Control Reference for LiteWing Flight Stabilizer

This document serves as a quick reference for the purpose and tuning of the PID (Proportional, Integral, Derivative) controllers used in the `dead-reckoning-maneuvers.py` script.

## Core PID Logic

The script uses a **Cascaded (Dual-Layer) PID** loop to control horizontal movement (X and Y axes).

### 1. Kp: The "Muscle" (Proportional)
*   **What it does:** Reacts to the **CURRENT** error.
*   **Position Kp:** Commands how fast the drone should fly back to the target based on distance.
*   **Velocity Kp:** Commands how much the drone should tilt to stop based on current speed.
*   **In Flight:** High Kp makes the drone feel "tight" and "locked in." If Kp is too high, the drone will shake or oscillate (shiver).

### 2. Ki: The "Patience" (Integral)
*   **What it does:** Reacts to the **HISTORY** of the error.
*   **Purpose:** Eliminates small, persistent offsets. If the drone consistently stays 5cm away from the target, Ki "builds up" power over a few seconds to push it those final 5cm.
*   **In Flight:** Necessary for correcting hardware imbalances (like off-center batteries). If Ki is too high, the drone will wander in slow, large waves.

### 3. Kd: The "Brakes" (Derivative)
*   **What it does:** Reacts to the **FUTURE** error (predictive).
*   **Purpose:** Acts as a shock absorber. It looks at the speed of approach and applies a counter-force to prevent the drone from flying past the target (Overshooting).
*   **In Flight:** Not used much in optical flow scripts because sensor noise can make Kd "chatter" (high frequency motor buzzing). We usually use internal smoothing instead.

---

## Safety and Integration Parameters

These parameters control the "guardrails" of the system, preventing the drone from reacting too violently or drifting away due to sensor noise.

| Parameter | Purpose | Why it's useful |
| :--- | :--- | :--- |
| **MAX_CORRECTION** | The **Speed Limit**. It limits the maximum tilt command sent to the drone (e.g., 0.7 units). | Prevents the drone from flipping over or making dangerous aggressive moves if the PID detects a large error. |
| **VELOCITY_THRESHOLD**| The **Noise Gate**. Any velocity smaller than this (e.g., 0.005 m/s) is ignored. | Prevents "jitter" from being integrated into the position. Without this, the drone's position would drift even when sitting on the floor. |
| **DRIFT_COMPENSATION**| The **Virtual Spring**. Slowly pulls the estimated position toward zero when moving very slowly. | Counteracts the naturally accumulating error of dead reckoning. It helps keep the "origin" from drifting away over long flights. |
| **PERIODIC_RESET** | The **Fresh Start**. Completely resets the integrated X/Y position to zero every few minutes (e.g., 90s). | Clears all accumulated drift error. It stops the drone from thinking it's across the room when it's actually still right in front of you. |
| **MAX_POSITION_ERROR**| The **Safety Clamp**. Limits how large the integrated position coordinate can grow (e.g., 2.0 meters). | Prevents "runaway" integration where the drone thinks it has flown 100 meters away and tries to fly back at full speed. |
| **VELOCITY_SMOOTHING**| The **Noise Filter**. Blends new velocity data with the previous reading (e.g., 0.85). | Smooths out raw sensor "jitter." High values (0.9+) are responsive but shaky; Low values (0.6-) are smooth but add flight lag. |

## Tuning Cheat Sheet

Use this guide to adjust your PID values in the GUI:

| Behavior | Cause | Fix |
| :--- | :--- | :--- |
| **Drone is "shivering" or vibrating** | Velocity Kp is too high | Decrease **Velocity Kp** |
| **Drone flies past the target and bounces back** | Position Kp is too high | Decrease **Position Kp** |
| **Drone feels "lazy" or slides on ice** | Velocity Kp is too low | Increase **Velocity Kp** |
| **Drone stops a few cm away from the target** | Position Ki is too low | Increase **Position Ki** |
| **Drone tries to fly away after stopping** | Ki is too high | Decrease **Ki** |

## Important Note on Height
**This script does NOT control Height PID.** 
Height is controlled by the drone's internal firmware. Up-and-down "bouncing" is usually caused by:
1. Low battery voltage.
2. Noisy floor surfaces (reflective tiles).
3. Propeller turbulence (Ground Effect).
