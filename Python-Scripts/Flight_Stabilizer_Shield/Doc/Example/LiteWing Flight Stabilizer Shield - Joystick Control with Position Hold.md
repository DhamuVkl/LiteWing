# **LiteWing Flight Stabilization Module - Joystick Control with Position Hold**

The LiteWing Flight Stabilizer Shield enables advanced flight control capabilities that combine manual joystick input with automatic position stabilization. In this guide, we will explore how to implement a control system where you can command the drone's movement using keyboard controls (WASD keys), and when you release those controls, the drone automatically maintains its position in space. This creates an intuitive flying experience where you get the precision of manual control when you need it, combined with the stability of autonomous hover when you don't.

The approach we're using here provides what many pilots consider the ideal balance for indoor flight. When you want the drone to move, you simply press and hold a direction key, and the drone smoothly travels in that direction. The moment you release the key, the drone doesn't just stop moving—it actively works to hold that exact position against any disturbances. The system achieves this by using dead reckoning to estimate position through integrating velocity measurements from the optical flow sensor, then applying PID control to maintain the estimated position and counteract drift.

## **System Overview**

The joystick position hold system we're implementing operates in two distinct modes that transition seamlessly based on your input. Understanding how these modes work and when the system switches between them is essential for getting the most out of this control scheme.

When you are actively pressing one or more of the WASD keys, the system operates in Joystick Mode. During this phase, you have direct velocity control over the drone. Pressing W commands forward motion, S commands backward motion, A moves left, and D moves right. The speed of this motion is determined by the sensitivity setting you configure. In this mode, the system temporarily disables position integration, allowing the drone to move freely according to your commands. The velocity commands you generate are sent directly to the flight controller without any position hold corrections applied.

The moment you release all directional keys, the system transitions into Position Hold Mode. At the instant of this transition, the system captures the drone's current estimated position and sets that as the new target position to maintain. From that point forward, PID controllers activate and begin working to counteract any drift from this held position. The optical flow sensor continues to measure velocity, and those measurements are fed into the control loop. The PID controllers compute correction commands that oppose any detected motion, effectively creating a virtual anchor point in space. This continues until you press a directional key again, at which point the system immediately returns to Joystick Mode.

This dual-mode behavior creates what we call a "fly-and-hold" control scheme. It's particularly useful for indoor navigation tasks where you might want to move to a specific location, inspect something while hovering steadily, then move to the next location. It's also an excellent tool for learning autonomous flight concepts because you can directly observe how the position hold controller responds to disturbances.

## **Hardware Setup**

Before we can work with the joystick position hold system, you need to ensure that your LiteWing drone is properly equipped with the Flight Stabilizer Shield and running compatible firmware. If you have not yet installed the shield or updated the drone's firmware to support the shield, you should refer to the [**Flight Stabilization Module User Guide**](https://docs.google.com/document/u/0/d/13HODtCuIMlrnUFQifmqlx-XEDWVDhSHalUJkYII49gc/edit). That document covers the complete hardware installation process and walks you through the firmware flashing steps. Make sure your drone is running the Shield-compatible firmware before continuing with this guide.

## **Software Setup**

Setting up your development environment correctly is critical for working with the position hold system. You need Python installed on your PC, along with the cflib library and its dependencies configured properly. Additionally, you must be able to establish a CRTP communication link between your PC and the LiteWing drone. Since cflib has a few prerequisites that must be met before it will function correctly, we recommend you refer to the [**LiteWing Python SDK Programming Guide**](https://circuitdigest.com/microcontroller-projects/how-to-program-litewing-drone-using-python-with-crazyflie-cflib-python-sdk). That document provides detailed instructions for installing dependencies, setting up cflib, connecting to the LiteWing Wi-Fi interface, and validating your communication setup with a simple test script. Completing that setup process is a necessary prerequisite before attempting to interact with any sensor on the Flight Stabilizer Module.

## **How Dead Reckoning Position Hold Works**

The position hold system we're implementing relies on a technique called dead reckoning. Unlike GPS-based systems that receive absolute position information from satellites, dead reckoning estimates position by integrating velocity measurements over time. This means we build up a relative position estimate starting from an arbitrary origin point, which we set at takeoff or whenever we reset the system.

The fundamental principle behind dead reckoning is simple. If you know how fast you're moving and in which direction, you can estimate how far you've traveled by multiplying velocity by time. The optical flow sensor provides us with velocity measurements in the X and Y directions. By integrating these velocities over time at each sensor update, we can estimate how far the drone has traveled from its starting position. Mathematically, this integration is expressed as:

[![](https://latex.codecogs.com/png.latex?x(t%20&plus;%20%5CDelta%20t)%20%3D%20x(t)%20&plus;%20v_x%20%5Ccdot%20%5CDelta%20t)](https://www.codecogs.com/eqnedit.php?latex=x(t%20&plus;%20%5CDelta%20t)%20%3D%20x(t)%20&plus;%20v_x%20%5Ccdot%20%5CDelta%20t#0)

[![](https://latex.codecogs.com/png.latex?y(t%20&plus;%20%5CDelta%20t)%20%3D%20y(t)%20&plus;%20v_y%20%5Ccdot%20%5CDelta%20t)](https://www.codecogs.com/eqnedit.php?latex=y(t%20&plus;%20%5CDelta%20t)%20%3D%20y(t)%20&plus;%20v_y%20%5Ccdot%20%5CDelta%20t#0)

In these equations, [![](https://latex.codecogs.com/png.latex?v_x)](https://www.codecogs.com/eqnedit.php?latex=v_x#0) and [![](https://latex.codecogs.com/png.latex?v_y)](https://www.codecogs.com/eqnedit.php?latex=v_y#0) represent the velocity in meters per second along each axis, and [![](https://latex.codecogs.com/png.latex?%5CDelta%20t)](https://www.codecogs.com/eqnedit.php?latex=%5CDelta%20t#0) is the time interval between samples. This integration runs continuously during flight, building up a position estimate relative to the takeoff point.

Once we have this position estimate, implementing position hold becomes straightforward. When you release all joystick inputs and the system transitions to Position Hold Mode, it captures the current estimated position and designates that as the target position to maintain. From that moment on, a series of calculations runs at approximately 50 times per second. First, the system computes the position error, which is simply the difference between where the drone should be (the target) and where it currently is (the estimated position). Second, it computes the velocity error, which is how fast the drone is currently moving (remember, we want zero velocity when holding position). These two error signals are fed into PID controllers that compute velocity correction commands. These corrections are sent to the flight controller, which adjusts the drone's thrust and tilt to counteract the detected errors.

The control law that governs this behavior combines both position error and velocity error. Position error tells us how far we've drifted from the target, and can be expressed as:

[![](https://latex.codecogs.com/png.latex?e_p%20%3D%20x_%7Btarget%7D%20-%20x_%7Bcurrent%7D)](https://www.codecogs.com/eqnedit.php?latex=e_p%20%3D%20x_%7Btarget%7D%20-%20x_%7Bcurrent%7D#0)

Velocity error tells us how fast we're moving when we should be stationary:

[![](https://latex.codecogs.com/png.latex?e_v%20%3D%200%20-%20v_%7Bcurrent%7D)](https://www.codecogs.com/eqnedit.php?latex=e_v%20%3D%200%20-%20v_%7Bcurrent%7D#0)

The correction command that gets sent to the drone combines contributions from both error signals:

[![](https://latex.codecogs.com/png.latex?v_%7Bcorrection%7D%20%3D%20K_p%20%5Ccdot%20e_p%20&plus;%20K_v%20%5Ccdot%20e_v)](https://www.codecogs.com/eqnedit.php?latex=v_%7Bcorrection%7D%20%3D%20K_p%20%5Ccdot%20e_p%20&plus;%20K_v%20%5Ccdot%20e_v#0)

This dual-loop approach is often called cascaded control. The position controller provides a restoring force that pulls the drone back toward the target position, while the velocity controller provides damping that prevents oscillation. Without the velocity damping, the position controller might cause the drone to overshoot the target position repeatedly, creating a bouncing behavior. The velocity term smooths this out.

One challenge inherent to all dead reckoning systems is drift accumulation. Small inaccuracies in velocity measurements, when integrated over time, result in position errors that grow continuously. A velocity measurement that is off by just 1 cm/s will cause a position error of 60 cm after one minute. To mitigate this problem without resorting to absolute position references, we implement two complementary mechanisms.

The first mechanism is velocity smoothing and thresholding. Raw optical flow data contains noise that varies from sample to sample. We filter this using an exponential moving average, which smooths out random fluctuations while maintaining responsiveness to real motion. Additionally, we apply a velocity threshold: any velocity measurement below a very small value (typically 0.005 m/s) is clamped to exactly zero. This prevents the system from integrating tiny noise values when the drone is actually stationary.

The second mechanism is integral decay. When the drone is nearly stationary (velocity magnitude close to zero), we apply a small decay term that gradually pulls the estimated position back toward the origin. This acts like a weak spring that slowly centers the position estimate. The decay is only applied during low-velocity conditions, so it doesn't interfere with tracking actual motion. The rate of decay is tunable, and the default value provides a good balance between drift suppression and tracking accuracy.

## **PID Control System**

The position hold implementation uses cascaded PID controllers for both position and velocity. Although the default configuration uses proportional-only control (what we call P-control), the full PID structure is available if you want to tune the system for optimal performance in your specific environment.

Understanding how these controllers work will help you tune them effectively. A PID controller computes its output as the sum of three terms: proportional, integral, and derivative. The proportional term responds to the current error magnitude. The integral term accumulates error over time and responds to persistent offsets. The derivative term responds to the rate of change of the error and provides predictive damping.

Our position PID controller attempts to drive the position error to zero. Its output is computed as:

[![](https://latex.codecogs.com/png.latex?u_%7Bpos%7D%20%3D%20K_%7Bp%7D%20e_p%20&plus;%20K_%7Bi%7D%20%5Cint%20e_p%20%5C%2C%20dt%20&plus;%20K_%7Bd%7D%20%5Cfrac%7Bde_p%7D%7Bdt%7D)](https://www.codecogs.com/eqnedit.php?latex=u_%7Bpos%7D%20%3D%20K_%7Bp%7D%20e_p%20&plus;%20K_%7Bi%7D%20%5Cint%20e_p%20%5C%2C%20dt%20&plus;%20K_%7Bd%7D%20%5Cfrac%7Bde_p%7D%7Bdt%7D#0)

The default gains for the position controller are shown in this table:

| Gain | Default Value | Effect |
|:---:|:---:|:-----|
| **POSITION_KP** | 1.2 | Main restoring force toward target position |
| **POSITION_KI** | 0.0 | Eliminates steady-state offset (disabled by default) |
| **POSITION_KD** | 0.0 | Provides additional damping (disabled by default) |

The proportional term provides the main restoring force. A higher proportional gain creates stronger position correction, pulling the drone back toward the target more aggressively. However, if you set this gain too high, the system becomes unstable and oscillates. Too low, and the drone drifts away from the target position without sufficient correction.

Our velocity PID controller provides active damping by opposing any residual velocity. Its output is computed as:

[![](https://latex.codecogs.com/png.latex?u_%7Bvel%7D%20%3D%20K_%7Bp%7D%20e_v%20&plus;%20K_%7Bi%7D%20%5Cint%20e_v%20%5C%2C%20dt%20&plus;%20K_%7Bd%7D%20%5Cfrac%7Bde_v%7D%7Bdt%7D)](https://www.codecogs.com/eqnedit.php?latex=u_%7Bvel%7D%20%3D%20K_%7Bp%7D%20e_v%20&plus;%20K_%7Bi%7D%20%5Cint%20e_v%20%5C%2C%20dt%20&plus;%20K_%7Bd%7D%20%5Cfrac%7Bde_v%7D%7Bdt%7D#0)

The default gains for the velocity controller are:

| Gain | Default Value | Effect |
|:---:|:---:|:-----|
| **VELOCITY_KP** | 1.2 | Active damping to prevent overshoot |
| **VELOCITY_KI** | 0.0 | Typically not needed |
| **VELOCITY_KD** | 0.0 | Typically not needed |

The velocity controller acts as active damping. When the position controller commands a correction, it creates motion. The velocity controller opposes that motion, preventing overshoot and oscillation. This is why we can use relatively high position gains without instability—the velocity loop provides the necessary damping.

The final velocity command sent to the drone's low-level controller combines both PID outputs:

```python
total_correction_vx = u_pos_x + u_vel_x
total_correction_vy = u_pos_y + u_vel_y
```

These corrections are clamped to safe limits to prevent aggressive maneuvers that could destabilize the drone. The default limit is ±0.1 m/s, which provides smooth, controlled corrections without sudden motions.

## **CRTP Communication for Position Hold**

The joystick position hold script communicates with the drone using the same CRTP log variables we introduced in the Optical Flow Sensor guide. We stream three primary variables at a rate of 100 Hz (every 10 milliseconds) to provide sufficient update frequency for the control loop.

| Variable | Type | Purpose |
|:---:|:---:|:-----|
| **motion.deltaX** | int16_t | Pixel shift in X direction from optical flow sensor |
| **motion.deltaY** | int16_t | Pixel shift in Y direction from optical flow sensor |
| **stateEstimate.z** | float | Fused altitude estimate in meters |

The motion deltas come directly from the PMW3901MB optical flow sensor and represent how many pixels the ground pattern has moved between consecutive sensor frames. The altitude value comes from the drone's state estimator, which fuses data from the VL53L1X Time-of-Flight sensor with barometric and IMU measurements to provide a stable height reading.

To convert the raw pixel deltas into velocities expressed in meters per second, we apply a geometry-based scaling calculation that accounts for the sensor's field of view, optical resolution, and sample rate. The conversion follows this formula:

```python
velocity_constant = (4.4 * DEG_TO_RAD) / (30.0 * DT)
velocity = delta_value * altitude * velocity_constant
```

The constant 4.4 represents the effective field of view in degrees. The value 30 corresponds to the optical resolution (how many pixels span that field of view). DT is the sample period in seconds, typically 0.01 for our 10ms logging interval. This scaling produces velocities in meters per second that we can use for position integration and control calculations.

## **Code Walkthrough**

The complete script implementation is available on the LiteWing GitHub repository under the **Python-Scripts/Flight_Stabilization_Module/** directory. You can access it at: [https://github.com/Circuit-Digest/LiteWing/tree/main/Python-Scripts/Flight_Stabilization_Module](https://github.com/Circuit-Digest/LiteWing/tree/main/Python-Scripts/Flight_Stabilization_Module)

The file we're examining is named **dead-reckoning-joystick-position-hold.py**. In the following sections, we'll walk through the key components of this implementation to understand how everything fits together.

### **Configuration Parameters**

The script begins with a comprehensive set of configuration parameters that control the drone's connection, flight behavior, control timing, and PID tuning. These parameters are defined as constants at the top of the file, making them easy to locate and modify without searching through the code.

```python
# Connection Configuration
DRONE_URI = "udp://192.168.43.42"
TARGET_HEIGHT = 0.4  # Target hover height in meters
TAKEOFF_TIME = 0.5   # Time allocated for takeoff sequence
LANDING_TIME = 0.5   # Time allocated for landing sequence
DEBUG_MODE = False   # Set to True to disable motors for testing

# Control Loop Timing
CONTROL_UPDATE_RATE = 0.02  # 50Hz control loop (20ms period)
SENSOR_PERIOD_MS = 10        # 100Hz sensor data streaming
DT = SENSOR_PERIOD_MS / 1000.0  # Convert to seconds

# Trim Corrections (manual bias adjustments)
TRIM_VX = 0.0
TRIM_VY = 0.0

# Position PID Controller Gains
POSITION_KP = 1.2
POSITION_KI = 0.0
POSITION_KD = 0.0

# Velocity PID Controller Gains
VELOCITY_KP = 1.2
VELOCITY_KI = 0.0
VELOCITY_KD = 0.0

# Control Limits and Safety
MAX_CORRECTION = 0.1              # Maximum velocity correction (m/s)
DRIFT_COMPENSATION_RATE = 0.15    # Position decay rate when stationary
MAX_POSITION_ERROR = 2.0          # Position estimate clamp limit

# Joystick Control
JOYSTICK_SENSITIVITY = 0.4  # Commanded velocity in manual mode (m/s)
```

Several of these parameters deserve special attention. The CONTROL_UPDATE_RATE determines how frequently the position hold calculations run. At 50 Hz (every 20ms), we get responsive corrections without overwhelming the drone's command interface. The TARGET_HEIGHT sets your preferred hover altitude after takeoff. TRIM_VX and TRIM_VY allow you to compensate for asymmetric flight characteristics—if your drone tends to drift in a particular direction even when commanded to hold, you can add a small bias here to counteract that tendency.

The PID gain parameters define the aggressiveness of the position hold controller. The default values work well for typical indoor environments with good optical flow tracking. The MAX_CORRECTION parameter acts as a safety limit, clamping the velocity commands to prevent the position controller from commanding dangerously aggressive maneuvers if it detects a large position error. JOYSTICK_SENSITIVITY controls how fast the drone moves when you press a directional key—increase this for faster manual flight, or decrease it for more precise, gentle movements.

### **Global State Variables**

The script maintains several categories of state variables that track the complete status of the control system from raw sensor readings through computed position to PID controller internal state. Understanding what these variables represent will help you follow the control flow.

```python
# Raw sensor data from the drone
current_height = 0.0
motion_delta_x = 0
motion_delta_y = 0
sensor_data_ready = False
current_battery_voltage = 0.0
battery_data_ready = False

# Computed velocities (m/s)
current_vx = 0.0
current_vy = 0.0
velocity_x_history = [0.0, 0.0]  # For smoothing filter
velocity_y_history = [0.0, 0.0]

# Integrated position estimate (dead reckoning)
integrated_position_x = 0.0
integrated_position_y = 0.0
last_integration_time = time.time()
position_integration_enabled = False

# PID controller internal state
position_integral_x = 0.0
position_integral_y = 0.0
last_position_error_x = 0.0
last_position_error_y = 0.0
velocity_integral_x = 0.0
velocity_integral_y = 0.0
last_velocity_error_x = 0.0
last_velocity_error_y = 0.0

# Target position for hold mode
target_position_x = 0.0
target_position_y = 0.0

# Control outputs sent to drone
current_correction_vx = 0.0
current_correction_vy = 0.0

# Mode flags
joystick_mode_active = False
```

The sensor data variables hold the most recent values received from the drone. The velocity variables contain both raw and smoothed velocity estimates computed from the optical flow deltas. The integrated position variables represent our dead reckoning estimate of where the drone is relative to the origin. The PID state variables store the integral accumulation and previous error values needed for the I and D terms. The target position variables define where we want to hold when in Position Hold Mode. Finally, the mode flag tells us whether we're currently in Joystick Mode or Position Hold Mode.

### **Velocity Calculation and Smoothing**

Raw optical flow deltas must be converted to velocities and smoothed before use. Two functions handle this processing.

```python
def calculate_velocity(delta_value, altitude):
    """Convert optical flow delta to linear velocity"""
    if altitude <= 0:
        return 0.0
    
    # Geometry constant based on sensor FOV and sample rate
    velocity_constant = (4.4 * DEG_TO_RAD) / (30.0 * DT)
    velocity = delta_value * altitude * velocity_constant
    return velocity
```

The calculate_velocity function takes a raw pixel delta from the optical flow sensor and the current altitude, then returns the corresponding linear velocity in meters per second. The calculation accounts for the sensor's 4.4-degree field of view, 30-pixel resolution, and our logging sample period. If altitude is zero or negative, the function returns zero velocity because we cannot reliably compute velocity without valid altitude information.

```python
def smooth_velocity(new_velocity, history):
    """Simple 2-point smoothing filter"""
    history[1] = history[0]
    history[0] = new_velocity
    alpha = VELOCITY_SMOOTHING_ALPHA  # Typically 0.9
    smoothed = (history[0] * alpha) + (history[1] * (1 - alpha))
    
    # Suppress very small velocities to prevent drift
    if abs(smoothed) < VELOCITY_THRESHOLD:
        smoothed = 0.0
    return smoothed
```

The smooth_velocity function implements a simple exponential moving average filter. We maintain a two-point history for each axis, and the smoothed output is a weighted combination of the current and previous values. The alpha parameter (typically 0.9) controls the balance between new and old data. A higher alpha value produces smoother output but with slightly more lag. After smoothing, we apply a threshold test: velocities smaller than a tiny value (0.005 m/s by default) are clamped to exactly zero. This prevents the integration of noise when the drone is actually stationary.

### **Position Integration**

Velocities are integrated over time to estimate position through dead reckoning. The integrate_position function handles this calculation along with drift compensation and bounds checking.

```python
def integrate_position(vx, vy, dt):
    """Dead reckoning: integrate velocity to position"""
    global integrated_position_x, integrated_position_y
    
    # Sanity check on time step
    if dt <= 0 or dt > 0.1:
        return
    
    # Basic integration step
    integrated_position_x += vx * dt
    integrated_position_y += vy * dt
    
    # Apply drift compensation when velocity magnitude is low
    velocity_magnitude = (vx * vx + vy * vy) ** 0.5
    if velocity_magnitude < VELOCITY_THRESHOLD * 2:
        integrated_position_x -= integrated_position_x * DRIFT_COMPENSATION_RATE * dt
        integrated_position_y -= integrated_position_y * DRIFT_COMPENSATION_RATE * dt
    
    # Clamp position to reasonable bounds
    integrated_position_x = max(-MAX_POSITION_ERROR, 
                                min(MAX_POSITION_ERROR, integrated_position_x))
    integrated_position_y = max(-MAX_POSITION_ERROR, 
                                min(MAX_POSITION_ERROR, integrated_position_y))
```

The function begins by validating the time step. If dt is zero, negative, or unreasonably large (which could happen if the logging temporarily paused), we skip the integration to avoid creating a large jump in the position estimate. The core integration is straightforward: we simply add the product of velocity and time to the current position.

The drift compensation mechanism activates when the velocity magnitude is very small, indicating the drone is nearly stationary. In this condition, we apply a small decay that gradually pulls the position estimate back toward zero. The decay rate is proportional to the current position error, so larger errors decay faster. This acts like a weak centering spring that helps counteract long-term drift without interfering with tracking real motion.

Finally, we clamp the position estimate to reasonable bounds. If sensor errors or other problems cause the integration to run away, the clamp prevents absurdly large position values. The default limit of ±2 meters is sufficient for typical indoor flight spaces.

### **Position Hold Reset**

When transitioning from Joystick Mode to Position Hold Mode, we need to reset the control system to treat the current location as the new target. The set_current_position_as_target function handles this critical transition.

```python
def set_current_position_as_target():
    """Reset position tracking to treat current location as new origin"""
    global integrated_position_x, integrated_position_y
    global target_position_x, target_position_y
    global position_integral_x, position_integral_y
    global velocity_integral_x, velocity_integral_y
    global last_position_error_x, last_position_error_y
    global last_velocity_error_x, last_velocity_error_y
    global current_correction_vx, current_correction_vy
    global velocity_x_history, velocity_y_history
    
    # Set current location as new origin
    integrated_position_x = 0.0
    integrated_position_y = 0.0
    target_position_x = 0.0
    target_position_y = 0.0
    
    # Reset all PID controller internal state
    position_integral_x = 0.0
    position_integral_y = 0.0
    last_position_error_x = 0.0
    last_position_error_y = 0.0
    velocity_integral_x = 0.0
    velocity_integral_y = 0.0
    last_velocity_error_x = 0.0
    last_velocity_error_y = 0.0
    
    # Clear any pending corrections
    current_correction_vx = 0.0
    current_correction_vy = 0.0
    
    # Reset velocity smoothing history
    velocity_x_history[0] = 0.0
    velocity_x_history[1] = 0.0
    velocity_y_history[0] = 0.0
    velocity_y_history[1] = 0.0
```

This function performs a complete state reset. We set both the current position estimate and the target position to zero, effectively redefining the origin at the drone's current location. All PID controller internal state gets cleared—this includes the integral accumulators and stored previous errors. If we didn't clear these, the controller would "remember" errors from the previous hold position and incorrectly apply them to the new hold position. We also clear any pending correction commands and reset the velocity smoothing history to prevent residual artifacts from affecting the new position hold attempt.

### **PID Control Calculation**

The calculate_position_hold_corrections function implements the core position hold logic by combining position and velocity PID controllers to compute the correction commands.

```python
def calculate_position_hold_corrections():
    """Calculate control corrections using PID controllers"""
    global current_correction_vx, current_correction_vy
    global position_integral_x, position_integral_y
    global last_position_error_x, last_position_error_y
    global velocity_integral_x, velocity_integral_y
    global last_velocity_error_x, last_velocity_error_y
    
    # Safety check
    if not sensor_data_ready or current_height <= 0:
        current_correction_vx = 0.0
        current_correction_vy = 0.0
        return 0.0, 0.0
    
    # Compute errors (negative because we want to reduce error)
    position_error_x = -(integrated_position_x - target_position_x)
    position_error_y = -(integrated_position_y - target_position_y)
    velocity_error_x = -current_vx
    velocity_error_y = -current_vy
    
    # Position PID - X axis
    position_p_x = position_error_x * POSITION_KP
    position_integral_x += position_error_x * CONTROL_UPDATE_RATE
    position_integral_x = max(-0.1, min(0.1, position_integral_x))
    position_i_x = position_integral_x * POSITION_KI
    position_derivative_x = (position_error_x - last_position_error_x) / CONTROL_UPDATE_RATE
    position_d_x = position_derivative_x * POSITION_KD
    last_position_error_x = position_error_x
    
    # Position PID - Y axis
    position_p_y = position_error_y * POSITION_KP
    position_integral_y += position_error_y * CONTROL_UPDATE_RATE
    position_integral_y = max(-0.1, min(0.1, position_integral_y))
    position_i_y = position_integral_y * POSITION_KI
    position_derivative_y = (position_error_y - last_position_error_y) / CONTROL_UPDATE_RATE
    position_d_y = position_derivative_y * POSITION_KD
    last_position_error_y = position_error_y
    
    # Velocity PID - X axis
    velocity_p_x = velocity_error_x * VELOCITY_KP
    velocity_integral_x += velocity_error_x * CONTROL_UPDATE_RATE
    velocity_integral_x = max(-0.05, min(0.05, velocity_integral_x))
    velocity_i_x = velocity_integral_x * VELOCITY_KI
    velocity_derivative_x = (velocity_error_x - last_velocity_error_x) / CONTROL_UPDATE_RATE
    velocity_d_x = velocity_derivative_x * VELOCITY_KD
    last_velocity_error_x = velocity_error_x
    
    # Velocity PID - Y axis
    velocity_p_y = velocity_error_y * VELOCITY_KP
    velocity_integral_y += velocity_error_y * CONTROL_UPDATE_RATE
    velocity_integral_y = max(-0.05, min(0.05, velocity_integral_y))
    velocity_i_y = velocity_integral_y * VELOCITY_KI
    velocity_derivative_y = (velocity_error_y - last_velocity_error_y) / CONTROL_UPDATE_RATE
    velocity_d_y = velocity_derivative_y * VELOCITY_KD
    last_velocity_error_y = velocity_error_y
    
    # Combine both controllers
    total_vx = (position_p_x + position_i_x + position_d_x) + \
               (velocity_p_x + velocity_i_x + velocity_d_x)
    total_vy = (position_p_y + position_i_y + position_d_y) + \
               (velocity_p_y + velocity_i_y + velocity_d_y)
    
    # Apply safety limits
    total_vx = max(-MAX_CORRECTION, min(MAX_CORRECTION, total_vx))
    total_vy = max(-MAX_CORRECTION, min(MAX_CORRECTION, total_vy))
    
    # Store for monitoring
    current_correction_vx = total_vx
    current_correction_vy = total_vy
    
    return total_vx, total_vy
```

This function implements the full cascaded PID architecture. We compute position errors by comparing where we are to where we want to be, and velocity errors by comparing our current velocity to the desired velocity of zero. For each axis, we calculate the P, I, and D terms of both the position and velocity controllers. The integral terms are accumulated over time and clamped to prevent windup. The derivative terms are computed as the rate of change of the error. Finally, we sum all contributions from both controllers and apply a safety clamp to ensure the commanded corrections stay within reasonable bounds.

### **Motion Sensor Callback**

The motion_callback function receives optical flow data from the drone and processes it into velocity and position estimates.

```python
def motion_callback(timestamp, data, logconf):
    """Motion sensor data callback"""
    global current_height, motion_delta_x, motion_delta_y
    global current_vx, current_vy, sensor_data_ready
    global last_integration_time
    
    # Extract sensor readings
    current_height = data.get("stateEstimate.z", 0)
    motion_delta_x = data.get("motion.deltaX", 0)
    motion_delta_y = data.get("motion.deltaY", 0)
    sensor_data_ready = True
    
    # Calculate velocities from optical flow deltas
    raw_velocity_x = calculate_velocity(motion_delta_x, current_height)
    raw_velocity_y = calculate_velocity(motion_delta_y, current_height)
    
    # Apply smoothing filter
    current_vx = smooth_velocity(raw_velocity_x, velocity_x_history)
    current_vy = smooth_velocity(raw_velocity_y, velocity_y_history)
    
    # Integrate position (only when NOT in joystick mode)
    current_time = time.time()
    dt = current_time - last_integration_time
    if 0.001 <= dt <= 0.1 and position_integration_enabled and not joystick_mode_active:
        integrate_position(current_vx, current_vy, dt)
    last_integration_time = current_time
    
    update_history()
```

This callback runs every time we receive a new packet of sensor data from the drone, which happens at 100 Hz. We extract the altitude and optical flow deltas, convert the deltas to velocities, apply smoothing, and check whether we should integrate the position. Note the condition that checks joystick_mode_active—when this flag is True (meaning you're actively pressing directional keys), we skip position integration. This allows you to move freely during manual control without the position estimate fighting against your commands.

### **Joystick Input Handling**

The GUI captures keyboard input through event handlers that update the key state flags.

```python
def on_key_press(self, event):
    """Handle key press events"""
    key = event.keysym.lower()
    if key in self.joystick_keys:
        self.joystick_keys[key] = True
        self.key_pressed_flags[key] = True

def on_key_release(self, event):
    """Handle key release events"""
    key = event.keysym.lower()
    if key in self.joystick_keys:
        self.joystick_keys[key] = False
```

These simple event handlers maintain a dictionary that tracks which directional keys are currently pressed. The control loop uses this dictionary to determine what velocity commands to send and whether to activate position hold.

### **Joystick Control Loop**

A separate thread continuously evaluates the joystick state and sends appropriate commands to the drone.

```python
def joystick_control_loop(self):
    """Main joystick control loop with position hold"""
    global joystick_mode_active
    
    while self.joystick_active and scf_instance is not None:
        # Check if any directional keys are pressed
        any_key_pressed = any(self.joystick_keys.values())
        
        if any_key_pressed:
            # ===== JOYSTICK MODE =====
            joystick_mode_active = True
            
            # Build velocity command from key states
            vx_cmd = 0.0
            vy_cmd = 0.0
            
            if self.joystick_keys['w']:  # Forward
                vy_cmd += JOYSTICK_SENSITIVITY
            if self.joystick_keys['s']:  # Backward
                vy_cmd -= JOYSTICK_SENSITIVITY
            if self.joystick_keys['a']:  # Left
                vx_cmd -= JOYSTICK_SENSITIVITY
            if self.joystick_keys['d']:  # Right
                vx_cmd += JOYSTICK_SENSITIVITY
            
            # Apply trim corrections
            vx_cmd += TRIM_VX
            vy_cmd += TRIM_VY
            
            # Send velocity command to drone
            cf.commander.send_velocity_world_setpoint(vx_cmd, vy_cmd, 0, 0)
            
        else:
            # ===== POSITION HOLD MODE =====
            if joystick_mode_active:
                # Just transitioned from joystick to hold
                set_current_position_as_target()
                joystick_mode_active = False
            
            # Calculate PID corrections
            corr_vx, corr_vy = calculate_position_hold_corrections()
            
            # Apply trim corrections
            corr_vx += TRIM_VX
            corr_vy += TRIM_VY
            
            # Send correction command to drone
            cf.commander.send_velocity_world_setpoint(corr_vx, corr_vy, 0, 0)
        
        # Run at control loop rate
        time.sleep(CONTROL_UPDATE_RATE)
```

This loop runs continuously at 50 Hz (every 20ms). On each iteration, it checks whether any directional keys are pressed. If yes, we're in Joystick Mode: we build a velocity command based on which keys are active, apply any trim corrections, and send that command to the drone. If no keys are pressed, we're in Position Hold Mode: we first check if we just transitioned into this mode (in which case we reset the target position), then we calculate PID corrections, apply trim, and send those corrections to the drone.

## **GUI and Visualization**

The script includes a comprehensive Tkinter-based graphical user interface that provides both control and monitoring capabilities. The interface is divided into several functional areas that let you adjust parameters, observe system state, and visualize the flight path.

The parameter adjustment panel allows you to modify critical system parameters without editing the code and restarting. You can change the target height, which determines how high the drone hovers after takeoff. The TRIM VX and TRIM VY parameters let you add small bias corrections if you notice the drone consistently drifts in a particular direction. The PID gain sliders (or entry boxes, depending on the implementation) allow you to adjust both the position and velocity controller gains. The joystick sensitivity control determines how fast the drone moves when you press the directional keys. All of these parameters can be adjusted in real-time, though some changes (particularly PID gains) take effect on the next control cycle.

The real-time status display section shows you the current values of important system variables. You can see the current height as measured by the sensors, the battery voltage (which helps you know when it's time to land), the estimated velocities in X and Y, the estimated position from dead reckoning, the computed correction values being sent to the drone, and the current flight phase. The flight phase indicator tells you whether the system is idle, taking off, in joystick control, holding position, or landing.

The trajectory visualization uses four matplotlib plots embedded in the Tkinter window. The first plot shows the estimated velocities in X and Y over time, giving you a sense of how the drone is moving. The second plot displays the integrated 2D position estimate—this is your dead reckoning trajectory with your current position marked by a red dot. The third plot shows the control corrections being applied by the PID controller, which helps you understand how hard the system is working to maintain position. The fourth plot tracks height over time with a reference line indicating your target altitude.

## **Tuning PID Parameters**

The default P-only control configuration works well for basic position hold in most environments, but you may want to tune the PID gains to optimize performance for your specific conditions. Understanding what each gain does and how to adjust it will help you achieve better position hold quality.

The position controller proportional gain (POSITION_KP) controls how aggressively the system tries to return to the target position. When you increase this gain, you create a stronger restoring force—the drone will more aggressively move back toward the target when it detects position error. However, if you set the gain too high, the system becomes unstable and begins to oscillate, bouncing back and forth around the target position. If you set it too low, the drone will drift away from the target without sufficient correction being applied to pull it back. A good starting range for this gain is 1.0 to 1.5, and the default value of 1.2 typically works well.

The position controller integral gain (POSITION_KI) accumulates position error over time and provides a correction proportional to that accumulation. This term is useful for eliminating steady-state errors—situations where the drone consistently stays offset from the target by a small, constant amount. However, integral terms can cause windup problems where the accumulation grows too large and creates instability. Use this gain sparingly, starting at 0.0 and increasing only if you observe persistent offset errors. Values in the range of 0.0 to 0.05 are typical.

The position controller derivative gain (POSITION_KD) responds to the rate of change of position error, providing predictive damping. In our cascaded architecture, this term is usually not needed because the velocity controller already provides damping. You can typically leave this at 0.0 unless you have specific performance requirements that justify adding derivative action.

The velocity controller proportional gain (VELOCITY_KP) provides active damping that prevents overshoot and oscillation. When the position controller commands a correction, it creates motion. The velocity controller opposes that motion in proportion to this gain, effectively acting as a brake that prevents the drone from flying past the target position. The default value of 1.2 provides good damping for most situations. Increase this if you see pronounced overshoot, or decrease it if the system feels sluggish or overly damped.

The velocity controller integral and derivative gains (VELOCITY_KI and VELOCITY_KD) are typically not needed in this configuration. You can leave both at 0.0 in most cases.

When tuning the PID controller, follow this systematic procedure. Start with the default P-only control where POSITION_KP equals 1.2, VELOCITY_KP equals 1.2, and all other gains are 0.0. Test the position hold performance by commanding the drone to hover, then releasing the controls and observing how well it maintains position. If you see visible oscillation where the drone bounces back and forth, reduce the position proportional gain by steps of 0.1 until the oscillation stops. If you see excessive drift where the drone slowly wanders away from the target position, increase the position proportional gain by steps of 0.1 until drift becomes acceptable. If after adjusting the proportional gain you still observe a persistent steady-state offset (the drone consistently sits offset from the target by a small, constant amount), add a small integral term by setting POSITION_KI to a value between 0.01 and 0.05. If you need more aggressive damping to prevent overshoot, increase VELOCITY_KP. Test each change thoroughly before making additional adjustments.

## **Testing the System**

Testing the joystick position hold system should follow a structured procedure that gradually introduces complexity. Begin with basic operation verification, then progress to manual control testing, and finally evaluate the position hold quality.

For basic operation, start by launching the script and clicking the "Sensor Test" button to connect to the drone and verify that all sensors are functioning correctly. The GUI will display status messages indicating whether the connection succeeded and whether the optical flow sensor and altitude sensor are providing data. Once the sensor test completes successfully, you can adjust the target height if you want to hover at an altitude different from the default 0.4 meters. When ready, click "Start Joystick Control" to initiate the autonomous takeoff sequence. The drone will rise to the target height and automatically enter Position Hold Mode.

Once the drone is hovering stably, you can test the manual control functionality. Press the W key and observe that the drone moves forward (in the positive Y direction relative to the drone's coordinate frame). Release the key and verify that the drone transitions smoothly into position hold, stopping its forward motion and maintaining that position. Repeat this test with the other directional keys: S for backward motion (negative Y), A for leftward motion (negative X), and D for rightward motion (positive X). The trajectory plot in the GUI should show your commanded path as a line trace, and when you release the controls, the position estimate should remain relatively stable.

To evaluate position hold quality, we need to consider what "good" performance looks like and identify signs of poor tuning or environmental issues. A well-tuned position hold system will keep the drone within approximately ±0.1 meters of the target position when no external disturbances are present. The velocity correction values displayed in the GUI should remain small, typically under 0.05 m/s, indicating that only gentle corrections are needed.
