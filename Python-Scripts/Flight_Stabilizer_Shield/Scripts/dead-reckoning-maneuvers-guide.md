# LiteWing Flight Stabilizer: Dead Reckoning Maneuvers Script Analysis

## 1. Overview
The `dead-reckoning-maneuvers.py` script is a comprehensive flight control system for the Crazyflie drone, specifically designed to use the **LiteWing Flight Stabilizer Shield**. It implements **Dead Reckoning** using an optical flow sensor to maintain position and perform precise maneuvers without relying on external positioning systems like GPS or motion capture.

---

## 2. Core Concepts
### Dead Reckoning
Dead reckoning is the process of calculating the current position of a moving object by using a previously determined position and advancing that position based on known or estimated speeds over elapsed time. In this script:
- **Velocity Estimation**: Derived from the optical flow sensor (bottom-facing camera) which measures the movement of pixels across the ground.
- **Integration**: The velocity ($m/s$) is multiplied by time ($dt$) and added to the previous position to estimate current $X$ and $Y$ coordinates.

### PID Control
The script uses a dual-loop PID (Proportional-Integral-Derivative) controller:
1.  **Position Loop (Outer)**: Calculates the velocity needed to correct the distance from the target position.
2.  **Velocity Loop (Inner)**: Damps the movement to prevent oscillations and ensures smooth velocity tracking.

---

## 3. Script Structure & Functionality

### A. Initialization & Configuration
- **Constants**: Defines PID gains (`POSITION_KP`, `VELOCITY_KP`, etc.), maneuver parameters, and sensor scaling.
- **Global State**: Tracks `integrated_position_x/y`, `current_vx/vy`, and the current `flight_phase`.
- **Threading**: Uses the `threading` module to run the GUI and the Flight Controller in parallel, ensuring the UI remains responsive.

### B. The GUI Class (`DeadReckoningGUI`)
The interface is built using `tkinter` and `matplotlib`:
- **Real-Time Monitors**: 
    - **2D Trajectory Plot**: Shows the path flown by the drone.
    - **Velocity Graphs**: Real-time X/Y speeds.
    - **Correction Graphs**: PID output values.
    - **Height Graph**: Altitude tracking.
- **Maneuver Buttons**: One-click commands for Forward, Backward, Left, Right, and Square patterns.
- **Tuning Panel**: Allows real-time adjustment of PID gains, Trim values, and Optical Flow scaling.
- **LED Control**: Interface for controlling the LiteWing Shield's NeoPixel LEDs.

### C. Data Acquisition (`motion_callback`)
This function is the heartbeat of the system, triggered every 10ms:
1.  **Data Extraction**: Reads `deltaX`, `deltaY` from the optical flow sensor and `z` (height) from the laser sensor.
2.  **Velocity Calculation**: Uses the formula `velocity = delta * height * constant` to convert pixel movement to physical movement.
3.  **Smoothing**: Applies a low-pass filter to remove sensor noise.
4.  **Integration**: Updates the `integrated_position` variables.

### D. Control Logic (`calculate_position_hold_corrections`)
Calculates the correction setpoints:
- **Error Calculation**: `error = target_position - current_position`.
- **PID Math**: Computes P, I, and D terms for both position and velocity.
- **Approach Factor**: Gradually reduces speed as the drone nears its target to prevent overshooting.
- **Safety Clamping**: Limits the maximum correction setpoint to prevent aggressive movements.

---

## 4. Flight Logic: Takeoff & Position Hold

When the **"Start Flight"** button is pressed, the following sequence occurs:

1.  **Safety Check (`start_flight`)**: 
    - At **Line 3244**, the script validates battery voltage and ensures sensor data is flowing. 
    - It then starts the `flight_controller_thread` in the background.

2.  **Takeoff Cycle (`flight_controller_thread`)**:
    - At **Line 3403**, the drone enters the `TAKEOFF` phase.
    - It commands the drone to reach `TARGET_HEIGHT` while simultaneously running the position-hold math (**Line 3434**) to ensure it doesn't drift sideways while going up.

3.  **The Brain behind Hold (`calculate_position_hold_corrections`)**:
    - Found at **Line 518**, this function is the "Math Engine". It compares the drone's estimated position (calculated via dead reckoning) to the target $(0,0)$ and generates the necessary thrust corrections.

4.  **Stabilization & Hover**:
    - After reaching the target height, the drone enters the `HOVER` phase (**Line 3749**), where it continuously applies these PID corrections to stay locked in place.

---

## 5. Key Features & Innovations

### 1. Robust Maneuver System
- **Single Leg Maneuvers**: Moves a fixed distance (e.g., 0.5m) in any direction and holds.
- **Square Pattern**: Executes a 4-waypoint sequence.
- **"Hop" Style Flight**: A unique feature where the drone lands at each waypoint. This "resets" the physical environment and can significantly improve the success rate of complex patterns in tight spaces.

### 2. Live Tuning & Debugging
- Users can tune the PID values *while the drone is flying*, observing immediate changes in the "Correction" and "Velocity" plots.
- **Debug Mode**: Allows running the script without spinning the motors, useful for testing sensor data and logic on a stationary drone.

### 3. Safety Mechanisms
- **Link Safety**: Automatically stops if the radio connection is lost.
- **Sensor Heartbeat**: Detects if the optical flow sensor stops sending data.
- **Height Validation**: Emergency stops if the height sensor appears frozen or stuck during takeoff. 
    - **Note**: This can be disabled by setting `ENABLE_HEIGHT_SENSOR_SAFETY = False` in the configuration section if you are testing in environments with irregular ground surfaces.
- **Low Battery Alert**: Blinks LEDs and logs warnings if voltage drops below a safe threshold.

---

## 6. How to Read the Code (File Flow)
1.  **Lines 1-231**: Imports and Global Variable Definitions.
2.  **Lines 232-432**: Communication and NeoPixel Helpers.
3.  **Lines 435-517**: The "Math" - Velocity, Smoothing, and Integration.
4.  **Lines 518-639**: The "Brain" - PID Control Logic.
5.  **Lines 665-841**: Data Logging Setup.
6.  **Lines 906-3822**: The `DeadReckoningGUI` class (UI definition + Flight Logic).
7.  **Lines 3333-3822**: The main `flight_controller_thread` (The sequencing logic).
8.  **Lines 4398-EOF**: Main script entry point which starts the Tkinter event loop.
