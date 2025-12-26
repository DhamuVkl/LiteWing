# **LiteWing Flight Stabilization Module - Optical Position Hold**

The LiteWing Flight Stabilizer Shield enables autonomous position hold capabilities using optical flow sensors combined with dead reckoning algorithms. This guide explores how to implement a fully autonomous position hold system where the drone takes off, hovers at a target altitude, and actively maintains its position in space without manual intervention. This creates a hands-free flying experience where the drone stabilizes itself against environmental disturbances and holds its position with minimal drift.

The optical position hold system represents the foundation of autonomous flight control. Unlike manual control where you constantly adjust the drone's position, or joystick-assisted control where you command movements, this system operates completely autonomously. Once you initiate the flight sequence, the drone takes off to the target height and immediately begins working to maintain that exact position. The system continuously measures velocity using the optical flow sensor, integrates these measurements to estimate position through dead reckoning, and applies PID control corrections to counteract any detected drift or motion. This approach is ideal for applications requiring stable hover, aerial photography, or as a learning platform for understanding autonomous flight control algorithms.

## **System Overview**

The optical position hold system operates through a continuous control loop that runs throughout the entire flight. Understanding how this loop functions and what calculations occur at each stage is essential for working with and tuning the system effectively.

When you start the position hold flight, the system first performs a sensor test phase to verify that all required sensors are functioning correctly. This includes checking the optical flow sensor for motion detection capability, verifying the Time-of-Flight sensor provides valid altitude readings, and confirming battery voltage is above the safe threshold. Once sensors pass validation, the system arms the drone and prepares for takeoff.

During the takeoff phase, the drone rises to the configured target height using simple hover commands. Position integration is not yet active during this phase—the system focuses solely on reaching the target altitude. Once the drone reaches the target height, it enters a stabilization phase where it allows a few seconds for any residual oscillations to settle. During stabilization, the position hold control system becomes active and begins calculating corrections, but these corrections are initially small as the drone is still settling from the ascent.

After stabilization completes, the system transitions into the main position hold phase. At this point, the position tracking origin is set to the drone's current location, and this becomes the target position that the system will work to maintain. From this moment forward, the control loop runs continuously at 50 Hz (every 20 milliseconds). On each iteration, the system reads the latest optical flow measurements, converts them to velocities, integrates velocity to estimate position, compares the estimated position against the target position, calculates position and velocity errors, feeds these errors through cascaded PID controllers, and sends the resulting correction commands to the drone's flight controller. This loop continues for the duration of the hover period, which is configurable but defaults to 60 seconds.

Throughout the position hold phase, the system employs several drift mitigation strategies. Small velocity measurements below a threshold are clamped to zero to prevent integrating sensor noise. When the drone is nearly stationary, a gentle decay pulls the position estimate gradually back toward the origin, counteracting long-term drift accumulation. Additionally, the system implements a periodic position reset that completely resets the integrated position to zero every 90 seconds. This prevents unbounded drift accumulation that is inevitable in any dead reckoning system. The position reset is transparent—the drone doesn't actually move, we simply redefine the current location as the new origin.

When the hover duration expires, the system transitions to landing. During landing, position hold corrections are disabled, and the drone descends using simple hover commands with a target height of zero. Once on the ground, the motors are stopped and the flight sequence completes.

## **Hardware Setup**

Before working with the optical position hold system, ensure your LiteWing drone is properly equipped with the Flight Stabilizer Shield and running Shield-compatible firmware. The shield must be correctly installed with all connections secure, including the optical flow sensor (PMW3901MB) and the Time-of-Flight altitude sensor (VL53L1X). If you have not yet installed the shield or updated the firmware, refer to the [**Flight Stabilization Module User Guide**](https://docs.google.com/document/u/0/d/13HODtCuIMlrnUFQifmqlx-XEDWVDhSHalUJkYII49gc/edit). That document provides complete hardware installation instructions and firmware flashing procedures.

## **Software Setup**

Setting up your development environment is critical for working with the position hold system. You need Python installed with the cflib library properly configured, and you must be able to establish CRTP communication with the LiteWing drone over UDP. The cflib library has specific prerequisites that must be met before it functions correctly. We recommend following the [**LiteWing Python SDK Programming Guide**](https://circuitdigest.com/microcontroller-projects/how-to-program-litewing-drone-using-python-with-crazyflie-cflib-python-sdk) for detailed setup instructions. That guide covers installing Python dependencies, setting up cflib, connecting to the LiteWing Wi-Fi interface, and validating your communication setup with test scripts. Completing that setup is a prerequisite for this guide.

## **How Dead Reckoning Position Hold Works**

The position hold system relies on dead reckoning, a technique that estimates position by integrating velocity measurements over time rather than receiving absolute position information from an external source like GPS. Dead reckoning builds up a relative position estimate starting from an arbitrary origin point that we define at a particular moment in the flight.

The core principle is straightforward. If you know how fast you're moving and in which direction, you can estimate how far you've traveled by multiplying velocity by time. The optical flow sensor provides velocity measurements in the X and Y directions (relative to the drone's body frame). By integrating these velocities at each sensor update, we estimate how far the drone has moved from its starting position. The mathematical expression for this integration is:

[![](https://latex.codecogs.com/png.latex?x(t%20&plus;%20%5CDelta%20t)%20%3D%20x(t)%20&plus;%20v_x%20%5Ccdot%20%5CDelta%20t)](https://www.codecogs.com/eqnedit.php?latex=x(t%20&plus;%20%5CDelta%20t)%20%3D%20x(t)%20&plus;%20v_x%20%5Ccdot%20%5CDelta%20t#0)

[![](https://latex.codecogs.com/png.latex?y(t%20&plus;%20%5CDelta%20t)%20%3D%20y(t)%20&plus;%20v_y%20%5Ccdot%20%5CDelta%20t)](https://www.codecogs.com/eqnedit.php?latex=y(t%20&plus;%20%5CDelta%20t)%20%3D%20y(t)%20&plus;%20v_y%20%5Ccdot%20%5CDelta%20t#0)

In these equations, [![](https://latex.codecogs.com/png.latex?v_x)](https://www.codecogs.com/eqnedit.php?latex=v_x#0) and [![](https://latex.codecogs.com/png.latex?v_y)](https://www.codecogs.com/eqnedit.php?latex=v_y#0) represent velocities in meters per second, and [![](https://latex.codecogs.com/png.latex?%5CDelta%20t)](https://www.codecogs.com/eqnedit.php?latex=%5CDelta%20t#0) is the time interval between measurements. This integration runs continuously during the position hold phase, building up an estimate of position relative to the point where integration began.

Once we have this position estimate, implementing position hold becomes conceptually simple. The system captures the current estimated position and treats it as the target position. From that point on, calculations run at approximately 50 Hz. First, we compute the position error, which is the difference between the target position and the current estimated position. Second, we compute the velocity error, which is how fast the drone is currently moving (we want zero velocity when holding position). These two error signals feed into PID controllers that compute velocity correction commands. The flight controller receives these corrections and adjusts thrust and tilt to counteract the detected errors.

The control law combines both position error and velocity error. Position error indicates how far we've drifted from the target:

[![](https://latex.codecogs.com/png.latex?e_p%20%3D%20x_%7Btarget%7D%20-%20x_%7Bcurrent%7D)](https://www.codecogs.com/eqnedit.php?latex=e_p%20%3D%20x_%7Btarget%7D%20-%20x_%7Bcurrent%7D#0)

Velocity error indicates how fast we're moving when we should be stationary:

[![](https://latex.codecogs.com/png.latex?e_v%20%3D%200%20-%20v_%7Bcurrent%7D)](https://www.codecogs.com/eqnedit.php?latex=e_v%20%3D%200%20-%20v_%7Bcurrent%7D#0)

The correction command combines contributions from both error signals:

[![](https://latex.codecogs.com/png.latex?v_%7Bcorrection%7D%20%3D%20K_p%20%5Ccdot%20e_p%20&plus;%20K_v%20%5Ccdot%20e_v)](https://www.codecogs.com/eqnedit.php?latex=v_%7Bcorrection%7D%20%3D%20K_p%20%5Ccdot%20e_p%20&plus;%20K_v%20%5Ccdot%20e_v#0)

This dual-loop approach, called cascaded control, provides both position restoration and velocity damping. The position controller generates a restoring force that pulls the drone back toward the target position. The velocity controller provides damping that prevents oscillation and overshoot. Without velocity damping, the position controller might cause the drone to overshoot repeatedly, creating a bouncing behavior. The velocity term smooths this response.

One fundamental challenge of all dead reckoning systems is drift accumulation. Small inaccuracies in velocity measurements, when integrated over time, result in position errors that grow continuously. A velocity measurement error of just 1 cm/s causes a position error of 60 cm after one minute. To mitigate this without relying on absolute position references, we implement complementary mechanisms.

The first mechanism is velocity smoothing and thresholding. Raw optical flow data contains noise that varies sample-to-sample. We filter this using an exponential moving average that smooths random fluctuations while maintaining responsiveness to real motion. Additionally, we apply a velocity threshold: any measurement below a very small value (typically 0.005 m/s) is clamped to exactly zero. This prevents integrating tiny noise values when the drone is actually stationary.

The second mechanism is integral decay. When the drone is nearly stationary (velocity magnitude close to zero), we apply a small decay term that gradually pulls the estimated position back toward the origin. This acts like a weak spring slowly centering the position estimate. Decay is only applied during low-velocity conditions, so it doesn't interfere with tracking actual motion. The decay rate is tunable, and the default provides a good balance between drift suppression and tracking accuracy.

The third mechanism is periodic position reset. Every 90 seconds, the system completely resets the integrated position to zero. This prevents unbounded drift accumulation. The reset is transparent—the drone doesn't move, we simply redefine the current location as the new origin and continue from there.

## **PID Control System**

The position hold implementation uses cascaded PID controllers for both position and velocity. Although the default configuration uses proportional-only control (P-control), the full PID structure is available for tuning to optimize performance in your specific environment.

Understanding how these controllers work helps with effective tuning. A PID controller computes its output as the sum of three terms: proportional, integral, and derivative. The proportional term responds to current error magnitude. The integral term accumulates error over time and responds to persistent offsets. The derivative term responds to the rate of change of error and provides predictive damping.

Our position PID controller attempts to drive position error to zero. Its output is:

[![](https://latex.codecogs.com/png.latex?u_%7Bpos%7D%20%3D%20K_%7Bp%7D%20e_p%20&plus;%20K_%7Bi%7D%20%5Cint%20e_p%20%5C%2C%20dt%20&plus;%20K_%7Bd%7D%20%5Cfrac%7Bde_p%7D%7Bdt%7D)](https://www.codecogs.com/eqnedit.php?latex=u_%7Bpos%7D%20%3D%20K_%7Bp%7D%20e_p%20&plus;%20K_%7Bi%7D%20%5Cint%20e_p%20%5C%2C%20dt%20&plus;%20K_%7Bd%7D%20%5Cfrac%7Bde_p%7D%7Bdt%7D#0)

The default gains for the position controller are:

| Gain | Default Value | Effect |
|:---:|:---:|:-----|
| **POSITION_KP** | 1.2 | Main restoring force toward target position |
| **POSITION_KI** | 0.0 | Eliminates steady-state offset (disabled by default) |
| **POSITION_KD** | 0.0 | Provides additional damping (disabled by default) |

The proportional term provides the main restoring force. Higher proportional gain creates stronger position correction, pulling the drone back toward the target more aggressively. However, excessive gain causes instability and oscillation. Insufficient gain allows drift without adequate correction. A good starting range is 1.0 to 1.5, with the default of 1.2 typically working well.

Our velocity PID controller provides active damping by opposing residual velocity. Its output is:

[![](https://latex.codecogs.com/png.latex?u_%7Bvel%7D%20%3D%20K_%7Bp%7D%20e_v%20&plus;%20K_%7Bi%7D%20%5Cint%20e_v%20%5C%2C%20dt%20&plus;%20K_%7Bd%7D%20%5Cfrac%7Bde_v%7D%7Bdt%7D)](https://www.codecogs.com/eqnedit.php?latex=u_%7Bvel%7D%20%3D%20K_%7Bp%7D%20e_v%20&plus;%20K_%7Bi%7D%20%5Cint%20e_v%20%5C%2C%20dt%20&plus;%20K_%7Bd%7D%20%5Cfrac%7Bde_v%7D%7Bdt%7D#0)

The default gains for the velocity controller are:

| Gain | Default Value | Effect |
|:---:|:---:|:-----|
| **VELOCITY_KP** | 1.2 | Active damping to prevent overshoot |
| **VELOCITY_KI** | 0.0 | Typically not needed |
| **VELOCITY_KD** | 0.0 | Typically not needed |

The velocity controller acts as active damping. When the position controller commands a correction, it creates motion. The velocity controller opposes that motion, preventing overshoot and oscillation. This is why we can use relatively high position gains without instability—the velocity loop provides necessary damping.

The final velocity command sent to the drone combines both PID outputs:

```python
total_correction_vx = u_pos_x + u_vel_x
total_correction_vy = u_pos_y + u_vel_y
```

These corrections are clamped to safe limits to prevent aggressive maneuvers that could destabilize the drone. The default limit is ±0.1 m/s, providing smooth, controlled corrections without sudden motions.

## **CRTP Communication for Position Hold**

The optical position hold script communicates with the drone using CRTP log variables that stream sensor data at high frequency. We request three primary variables at 100 Hz (every 10 milliseconds) to provide sufficient update frequency for the control loop.

| Variable | Type | Purpose |
|:---:|:---:|:-----|
| **motion.deltaX** | int16_t | Pixel shift in X direction from optical flow sensor |
| **motion.deltaY** | int16_t | Pixel shift in Y direction from optical flow sensor |
| **stateEstimate.z** | float | Fused altitude estimate in meters |

The motion deltas come directly from the PMW3901MB optical flow sensor and represent how many pixels the ground pattern has moved between consecutive sensor frames. The altitude value comes from the drone's state estimator, which fuses data from the VL53L1X Time-of-Flight sensor with barometric and IMU measurements to provide stable height readings.

To convert raw pixel deltas into velocities expressed in meters per second, we apply a geometry-based scaling calculation that accounts for the sensor's field of view, optical resolution, and sample rate. The conversion follows this formula:

```python
velocity_constant = (4.4 * DEG_TO_RAD) / (30.0 * DT)
velocity = delta_value * altitude * velocity_constant
```

The constant 4.4 represents the effective field of view in degrees. The value 30 corresponds to the optical resolution (how many pixels span that field of view). DT is the sample period in seconds, typically 0.01 for our 10ms logging interval. This scaling produces velocities in meters per second for position integration and control calculations.

## **Code Walkthrough**

The complete script implementation is available on the LiteWing GitHub repository under the **Python-Scripts/Flight_Stabilizer_Shield/** directory. You can access it at: [https://github.com/Circuit-Digest/LiteWing/tree/main/Python-Scripts/Flight_Stabilizer_Shield](https://github.com/Circuit-Digest/LiteWing/tree/main/Python-Scripts/Flight_Stabilizer_Shield)

The file we're examining is named **dead-reckoning-optical-position-hold.py**. In the following sections, we'll walk through the key components to understand how everything fits together.

### **Configuration Parameters**

The script begins with comprehensive configuration parameters controlling the drone's connection, flight behavior, control timing, and PID tuning. These parameters are defined as constants at the top of the file, making them easy to locate and modify.

```python
# Connection Configuration
DRONE_URI = "udp://192.168.43.42"
TARGET_HEIGHT = 0.3  # Target hover height in meters
TAKEOFF_TIME = 0.5   # Time allocated for takeoff sequence
HOVER_DURATION = 60.0  # How long to hover with position hold
LANDING_TIME = 0.5   # Time allocated for landing sequence
DEBUG_MODE = False   # Set to True to disable motors for testing

# Control Loop Timing
VELOCITY_SMOOTHING_ALPHA = 0.9  # Filtering strength for velocity
VELOCITY_THRESHOLD = 0.005       # Consider drone "stationary" below this
CONTROL_UPDATE_RATE = 0.02       # 50Hz control loop (20ms period)
SENSOR_PERIOD_MS = 10            # 100Hz sensor data streaming
DT = SENSOR_PERIOD_MS / 1000.0   # Convert to seconds

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
DRIFT_COMPENSATION_RATE = 0.004   # Position decay rate when stationary
MAX_POSITION_ERROR = 2.0          # Position estimate clamp limit
PERIODIC_RESET_INTERVAL = 90.0    # Reset integrated position periodically
```

Several parameters deserve special attention. The CONTROL_UPDATE_RATE determines how frequently position hold calculations run. At 50 Hz (every 20ms), we get responsive corrections without overwhelming the drone's command interface. TARGET_HEIGHT sets your preferred hover altitude after takeoff. TRIM_VX and TRIM_VY allow compensating for asymmetric flight characteristics—if your drone tends to drift in a particular direction even when commanded to hold, you can add a small bias to counteract that tendency.

The PID gain parameters define the aggressiveness of the position hold controller. Default values work well for typical indoor environments with good optical flow tracking. The MAX_CORRECTION parameter acts as a safety limit, clamping velocity commands to prevent the position controller from commanding dangerously aggressive maneuvers if it detects large position error. PERIODIC_RESET_INTERVAL controls how often the position estimate is reset to prevent unbounded drift accumulation.

### **Global State Variables**

The script maintains several categories of state variables tracking the complete control system status from raw sensor readings through computed position to PID controller internal state. Understanding what these variables represent helps you follow the control flow.

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
last_reset_time = time.time()
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

# Flight state
flight_phase = "IDLE"
flight_active = False
```

The sensor data variables hold the most recent values received from the drone. Velocity variables contain both raw and smoothed velocity estimates computed from optical flow deltas. Integrated position variables represent our dead reckoning estimate of where the drone is relative to the origin. PID state variables store integral accumulation and previous error values needed for I and D terms. Target position variables define where we want to hold. Finally, flight state variables track the current phase of operation.

### **Velocity Calculation and Smoothing**

Raw optical flow deltas must be converted to velocities and smoothed before use. Two functions handle this processing.

```python
def calculate_velocity(delta_value, altitude):
    """Convert optical flow delta to linear velocity"""
    if altitude <= 0:
        return 0.0
    if USE_HEIGHT_SCALING:
        velocity_constant = (4.4 * DEG_TO_RAD) / (30.0 * DT)
        velocity = delta_value * altitude * velocity_constant
    else:
        velocity = delta_value * OPTICAL_FLOW_SCALE * DT
    return velocity
```

The calculate_velocity function takes a raw pixel delta from the optical flow sensor and current altitude, then returns the corresponding linear velocity in meters per second. The calculation accounts for the sensor's 4.4-degree field of view, 30-pixel resolution, and logging sample period. If altitude is zero or negative, the function returns zero velocity because we cannot reliably compute velocity without valid altitude information.

```python
def smooth_velocity(new_velocity, history):
    """Simple 2-point smoothing filter with adjustable alpha"""
    history[1] = history[0]
    history[0] = new_velocity
    alpha = VELOCITY_SMOOTHING_ALPHA
    smoothed = (history[0] * alpha) + (history[1] * (1 - alpha))
    if abs(smoothed) < VELOCITY_THRESHOLD:
        smoothed = 0.0
    return smoothed
```

The smooth_velocity function implements a simple exponential moving average filter. We maintain a two-point history for each axis, and the smoothed output is a weighted combination of current and previous values. The alpha parameter (typically 0.9) controls the balance between new and old data. Higher alpha produces smoother output with slightly more lag. After smoothing, we apply a threshold test: velocities smaller than 0.005 m/s are clamped to exactly zero. This prevents integrating noise when the drone is actually stationary.

### **Position Integration**

Velocities are integrated over time to estimate position through dead reckoning. The integrate_position function handles this calculation along with drift compensation and bounds checking.

```python
def integrate_position(vx, vy, dt):
    """Dead reckoning: integrate velocity to position"""
    global integrated_position_x, integrated_position_y
    
    # Sanity check on time step
    if dt <= 0 or dt > 0.1:
        return
    
    # Simple integration
    integrated_position_x += vx * dt
    integrated_position_y += vy * dt
    
    # Apply drift compensation when moving slowly
    velocity_magnitude = (vx * vx + vy * vy) ** 0.5
    if velocity_magnitude < VELOCITY_THRESHOLD * 2:
        integrated_position_x -= integrated_position_x * DRIFT_COMPENSATION_RATE * dt
        integrated_position_y -= integrated_position_y * DRIFT_COMPENSATION_RATE * dt
    
    # Clamp position error
    integrated_position_x = max(-MAX_POSITION_ERROR, 
                                min(MAX_POSITION_ERROR, integrated_position_x))
    integrated_position_y = max(-MAX_POSITION_ERROR, 
                                min(MAX_POSITION_ERROR, integrated_position_y))
```

The function begins by validating the time step. If dt is zero, negative, or unreasonably large (which could happen if logging temporarily paused), we skip integration to avoid creating a large jump in the position estimate. The core integration is straightforward: we add the product of velocity and time to the current position.

The drift compensation mechanism activates when velocity magnitude is very small, indicating the drone is nearly stationary. In this condition, we apply a small decay that gradually pulls the position estimate back toward zero. The decay rate is proportional to current position error, so larger errors decay faster. This acts like a weak centering spring that helps counteract long-term drift without interfering with tracking real motion.

Finally, we clamp the position estimate to reasonable bounds. If sensor errors or other problems cause integration to run away, the clamp prevents absurdly large position values. The default limit of ±2 meters is sufficient for typical indoor flight spaces.

### **Periodic Position Reset**

To prevent unbounded drift accumulation in the dead reckoning system, we implement periodic position resets that completely zero the integrated position estimate at regular intervals.

```python
def periodic_position_reset():
    """Reset integrated position periodically to prevent drift accumulation"""
    global integrated_position_x, integrated_position_y, last_reset_time
    current_time = time.time()
    if current_time - last_reset_time >= PERIODIC_RESET_INTERVAL:
        integrated_position_x = 0.0
        integrated_position_y = 0.0
        last_reset_time = current_time
        return True
    return False
```

This function checks if the configured reset interval has elapsed since the last reset. By default, this interval is 90 seconds. When the interval expires, we reset both position estimates to zero and update the last reset timestamp. The function returns True when a reset occurs, allowing the calling code to log this event. The reset is transparent to the physical drone—it doesn't move, we simply redefine the current location as the new origin and continue position hold from there.

### **Position Hold Reset**

When starting position hold or transitioning between flight phases, we need to reset the control system state to ensure clean initialization.

```python
def reset_position_tracking():
    """Reset integrated position tracking"""
    global integrated_position_x, integrated_position_y
    global last_integration_time, last_reset_time
    global position_integration_enabled
    global position_integral_x, position_integral_y
    global velocity_integral_x, velocity_integral_y
    global last_position_error_x, last_position_error_y
    global last_velocity_error_x, last_velocity_error_y
    global target_position_x, target_position_y
    
    integrated_position_x = 0.0
    integrated_position_y = 0.0
    target_position_x = 0.0
    target_position_y = 0.0
    last_integration_time = time.time()
    last_reset_time = time.time()
    position_integration_enabled = True
    
    # Reset PID state
    position_integral_x = 0.0
    position_integral_y = 0.0
    velocity_integral_x = 0.0
    velocity_integral_y = 0.0
    last_position_error_x = 0.0
    last_position_error_y = 0.0
    last_velocity_error_x = 0.0
    last_velocity_error_y = 0.0
```

This function performs a complete state reset. We set both current position estimate and target position to zero, effectively defining the origin at the drone's current location. All PID controller internal state gets cleared—including integral accumulators and stored previous errors. If we didn't clear these, the controller would "remember" errors from previous flights and incorrectly apply them to the new position hold attempt. We also enable position integration so the system begins tracking position immediately.

### **PID Control Calculation**

The calculate_position_hold_corrections function implements the core position hold logic by combining position and velocity PID controllers to compute correction commands.

```python
def calculate_position_hold_corrections():
    """Calculate control corrections using PID controllers"""
    global current_correction_vx, current_correction_vy
    global position_integral_x, position_integral_y
    global last_position_error_x, last_position_error_y
    global velocity_integral_x, velocity_integral_y
    global last_velocity_error_x, last_velocity_error_y
    
    if not sensor_data_ready or current_height <= 0:
        current_correction_vx = 0.0
        current_correction_vy = 0.0
        return 0.0, 0.0
    
    # Calculate position errors (negative because we want to correct toward target)
    position_error_x = -(integrated_position_x - target_position_x)
    position_error_y = -(integrated_position_y - target_position_y)
    
    # Calculate velocity errors (negative because we want to dampen velocity)
    velocity_error_x = -current_vx
    velocity_error_y = -current_vy
    
    # Position PID Controller
    # Proportional
    position_p_x = position_error_x * POSITION_KP
    position_p_y = position_error_y * POSITION_KP
    # Integral (with anti-windup)
    position_integral_x += position_error_x * CONTROL_UPDATE_RATE
    position_integral_y += position_error_y * CONTROL_UPDATE_RATE
    position_integral_x = max(-0.1, min(0.1, position_integral_x))
    position_integral_y = max(-0.1, min(0.1, position_integral_y))
    position_i_x = position_integral_x * POSITION_KI
    position_i_y = position_integral_y * POSITION_KI
    # Derivative
    position_derivative_x = (position_error_x - last_position_error_x) / CONTROL_UPDATE_RATE
    position_derivative_y = (position_error_y - last_position_error_y) / CONTROL_UPDATE_RATE
    position_d_x = position_derivative_x * POSITION_KD
    position_d_y = position_derivative_y * POSITION_KD
    last_position_error_x = position_error_x
    last_position_error_y = position_error_y
    
    # Velocity PID Controller
    # Proportional
    velocity_p_x = velocity_error_x * VELOCITY_KP
    velocity_p_y = velocity_error_y * VELOCITY_KP
    # Integral (with anti-windup)
    velocity_integral_x += velocity_error_x * CONTROL_UPDATE_RATE
    velocity_integral_y += velocity_error_y * CONTROL_UPDATE_RATE
    velocity_integral_x = max(-0.05, min(0.05, velocity_integral_x))
    velocity_integral_y = max(-0.05, min(0.05, velocity_integral_y))
    velocity_i_x = velocity_integral_x * VELOCITY_KI
    velocity_i_y = velocity_integral_y * VELOCITY_KI
    # Derivative
    velocity_derivative_x = (velocity_error_x - last_velocity_error_x) / CONTROL_UPDATE_RATE
    velocity_derivative_y = (velocity_error_y - last_velocity_error_y) / CONTROL_UPDATE_RATE
    velocity_d_x = velocity_derivative_x * VELOCITY_KD
    velocity_d_y = velocity_derivative_y * VELOCITY_KD
    last_velocity_error_x = velocity_error_x
    last_velocity_error_y = velocity_error_y
    
    # Combine PID outputs
    position_correction_vx = position_p_x + position_i_x + position_d_x
    position_correction_vy = position_p_y + position_i_y + position_d_y
    velocity_correction_vx = velocity_p_x + velocity_i_x + velocity_d_x
    velocity_correction_vy = velocity_p_y + velocity_i_y + velocity_d_y
    
    # Total corrections
    total_vx = position_correction_vx + velocity_correction_vx
    total_vy = position_correction_vy + velocity_correction_vy
    
    # Apply limits
    total_vx = max(-MAX_CORRECTION, min(MAX_CORRECTION, total_vx))
    total_vy = max(-MAX_CORRECTION, min(MAX_CORRECTION, total_vy))
    
    # Store for GUI display
    current_correction_vx = total_vx
    current_correction_vy = total_vy
    return total_vx, total_vy
```

This function implements the full cascaded PID architecture. We compute position errors by comparing where we are to where we want to be, and velocity errors by comparing current velocity to the desired velocity of zero. For each axis, we calculate the P, I, and D terms of both position and velocity controllers. Integral terms are accumulated over time and clamped to prevent windup. Derivative terms are computed as the rate of change of error. Finally, we sum all contributions from both controllers and apply a safety clamp to ensure commanded corrections stay within reasonable bounds.

### **Motion Sensor Callback**

The motion_callback function receives optical flow data from the drone and processes it into velocity and position estimates.

```python
def motion_callback(timestamp, data, logconf):
    """Motion sensor data callback"""
    global current_height, motion_delta_x, motion_delta_y, sensor_data_ready
    global current_vx, current_vy, last_integration_time
    
    # Get sensor data
    current_height = data.get("stateEstimate.z", 0)
    motion_delta_x = data.get("motion.deltaX", 0)
    motion_delta_y = data.get("motion.deltaY", 0)
    sensor_data_ready = True
    
    # Calculate velocities
    raw_velocity_x = calculate_velocity(motion_delta_x, current_height)
    raw_velocity_y = calculate_velocity(motion_delta_y, current_height)
    
    # Apply smoothing
    current_vx = smooth_velocity(raw_velocity_x, velocity_x_history)
    current_vy = smooth_velocity(raw_velocity_y, velocity_y_history)
    
    # Dead reckoning position integration
    current_time = time.time()
    dt = current_time - last_integration_time
    if 0.001 <= dt <= 0.1 and position_integration_enabled:
        integrate_position(current_vx, current_vy, dt)
    last_integration_time = current_time
    
    # Update history for GUI
    update_history()
```

This callback runs every time we receive new sensor data from the drone, which happens at 100 Hz. We extract altitude and optical flow deltas, convert deltas to velocities, apply smoothing, and integrate position. The time step validation ensures we only integrate when dt is reasonable—neither too small (which could indicate a duplicate packet) nor too large (which could indicate a communication gap). Position integration only occurs when position_integration_enabled is True, allowing us to disable integration during certain flight phases like takeoff and landing.

### **Flight Control Loop**

The main flight control loop executes the complete position hold sequence from takeoff through landing.

```python
def flight_thread_func(self):
    """Position hold flight thread"""
    global flight_active, flight_phase
    global position_integration_enabled
    
    # Initialize
    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache="./cache")
    
    try:
        with SyncCrazyflie(DRONE_URI, cf=cf) as scf:
            flight_active = True
            
            # Setup logging
            log_motion, log_battery = setup_logging(cf, logger=self.log_to_output)
            time.sleep(1.0)
            
            # Reset position tracking
            reset_position_tracking()
            
            # Enable high-level commander
            if not DEBUG_MODE:
                cf.commander.send_setpoint(0, 0, 0, 0)
                time.sleep(0.1)
                cf.param.set_value("commander.enHighLevel", "1")
                time.sleep(0.5)
            
            # Takeoff
            flight_phase = "TAKEOFF"
            init_csv_logging(logger=self.log_to_output)
            start_time_local = time.time()
            
            while time.time() - start_time_local < TAKEOFF_TIME and flight_active:
                if not DEBUG_MODE:
                    cf.commander.send_hover_setpoint(TRIM_VX, TRIM_VY, 0, TARGET_HEIGHT)
                log_to_csv()
                time.sleep(0.01)
            
            # Stabilization
            flight_phase = "STABILIZING"
            stabilization_start = time.time()
            while time.time() - stabilization_start < 3.0 and flight_active:
                motion_vx, motion_vy = calculate_position_hold_corrections()
                log_to_csv()
                total_vx = TRIM_VX + motion_vy
                total_vy = TRIM_VY + motion_vx
                if not DEBUG_MODE:
                    cf.commander.send_hover_setpoint(total_vx, total_vy, 0, TARGET_HEIGHT)
                time.sleep(CONTROL_UPDATE_RATE)
            
            # Position Hold
            flight_phase = "POSITION_HOLD"
            hover_start = time.time()
            self.log_to_output(f"Position Hold active for {HOVER_DURATION:.0f}s")
            
            while time.time() - hover_start < HOVER_DURATION and flight_active:
                motion_vx, motion_vy = calculate_position_hold_corrections()
                if periodic_position_reset():
                    flight_phase = "POSITION_HOLD (RESET)"
                    self.log_to_output("Position reset to origin")
                else:
                    flight_phase = "POSITION_HOLD"
                
                log_to_csv()
                total_vx = TRIM_VX + motion_vy
                total_vy = TRIM_VY + motion_vx
                if not DEBUG_MODE:
                    cf.commander.send_hover_setpoint(total_vx, total_vy, 0, TARGET_HEIGHT)
                time.sleep(CONTROL_UPDATE_RATE)
            
            # Landing
            flight_phase = "LANDING"
            landing_start = time.time()
            while time.time() - landing_start < LANDING_TIME and flight_active:
                if not DEBUG_MODE:
                    cf.commander.send_hover_setpoint(TRIM_VX, TRIM_VY, 0, 0)
                log_to_csv()
                time.sleep(0.01)
            
            if not DEBUG_MODE:
                cf.commander.send_setpoint(0, 0, 0, 0)
            flight_phase = "COMPLETE"
            
    except Exception as e:
        flight_phase = "ERROR"
        self.log_to_output(f"Error: {str(e)}")
    finally:
        close_csv_logging(logger=self.log_to_output)
        flight_active = False
```

This function executes the complete flight sequence. It starts by initializing CRTP drivers and establishing a connection to the drone. After setting up sensor logging and enabling the high-level commander, it executes four distinct flight phases. During takeoff, the drone ascends to the target height using simple hover commands without position hold corrections. During stabilization, position hold calculations begin but the drone is still settling from the ascent. During the main position hold phase, the full control loop runs continuously, calculating corrections at 50 Hz and periodically resetting the position estimate. During landing, position hold is disabled and the drone descends using simple hover commands. Finally, motors are stopped and the flight completes.

## **GUI and Visualization**

The script includes a comprehensive Tkinter-based graphical user interface that provides both control and monitoring capabilities. The interface is divided into several functional areas for parameter adjustment, system state observation, and flight path visualization.

The parameter adjustment panel allows modifying critical system parameters without editing code and restarting. You can change the target height, which determines how high the drone hovers after takeoff. The hover duration controls how long the position hold phase lasts. TRIM VX and TRIM VY parameters let you add small bias corrections if the drone consistently drifts in a particular direction. The PID gain controls allow adjusting both position and velocity controller gains. The "Apply All Values" button commits your parameter changes to the running system.

The real-time status display section shows current values of important system variables. You can see current height as measured by sensors, battery voltage (helps you know when to land), estimated velocities in X and Y, estimated position from dead reckoning, computed correction values being sent to the drone, and current flight phase. The flight phase indicator tells you whether the system is idle, in sensor test, taking off, stabilizing, holding position, or landing.

The trajectory visualization uses four matplotlib plots embedded in the Tkinter window. The first plot shows estimated velocities in X and Y over time, giving you a sense of how the drone is moving. The second plot displays the integrated 2D position estimate—this is your dead reckoning trajectory with current position marked by a red dot and the target position (origin) marked in yellow. The third plot shows control corrections being applied by the PID controller, helping you understand how hard the system is working to maintain position. The fourth plot tracks height over time with a reference line indicating your target altitude.

The sensor test functionality provides a way to verify all sensors are working correctly before flight. When you click "Sensor Test", the system connects to the drone, sets up sensor logging, and begins streaming data without starting the motors. This allows you to confirm the optical flow sensor is detecting motion (move the drone manually and watch velocity values change), the altitude sensor provides valid readings, and battery voltage is above the safe threshold. Once sensors are verified, you can proceed with the position hold flight.

## **Tuning PID Parameters**

The default P-only control configuration works well for basic position hold in most environments, but you may want to tune PID gains to optimize performance for your specific conditions. Understanding what each gain does and how to adjust it helps you achieve better position hold quality.

The position controller proportional gain (POSITION_KP) controls how aggressively the system tries to return to the target position. When you increase this gain, you create a stronger restoring force—the drone will more aggressively move back toward the target when it detects position error. However, if you set the gain too high, the system becomes unstable and begins to oscillate, bouncing back and forth around the target position. If you set it too low, the drone will drift away from the target without sufficient correction being applied to pull it back. A good starting range for this gain is 1.0 to 1.5, and the default value of 1.2 typically works well.

The position controller integral gain (POSITION_KI) accumulates position error over time and provides a correction proportional to that accumulation. This term is useful for eliminating steady-state errors—situations where the drone consistently stays offset from the target by a small, constant amount. However, integral terms can cause windup problems where the accumulation grows too large and creates instability. Use this gain sparingly, starting at 0.0 and increasing only if you observe persistent offset errors. Values in the range of 0.0 to 0.05 are typical.

The position controller derivative gain (POSITION_KD) responds to the rate of change of position error, providing predictive damping. In our cascaded architecture, this term is usually not needed because the velocity controller already provides damping. You can typically leave this at 0.0 unless you have specific performance requirements that justify adding derivative action.

The velocity controller proportional gain (VELOCITY_KP) provides active damping that prevents overshoot and oscillation. When the position controller commands a correction, it creates motion. The velocity controller opposes that motion in proportion to this gain, effectively acting as a brake that prevents the drone from flying past the target position. The default value of 1.2 provides good damping for most situations. Increase this if you see pronounced overshoot, or decrease it if the system feels sluggish or overly damped.

The velocity controller integral and derivative gains (VELOCITY_KI and VELOCITY_KD) are typically not needed in this configuration. You can leave both at 0.0 in most cases.

When tuning the PID controller, follow this systematic procedure. Start with the default P-only control where POSITION_KP equals 1.2, VELOCITY_KP equals 1.2, and all other gains are 0.0. Test the position hold performance by running the script and observing how well it maintains position during the hover phase. If you see visible oscillation where the drone bounces back and forth, reduce the position proportional gain by steps of 0.1 until oscillation stops. If you see excessive drift where the drone slowly wanders away from the target position, increase the position proportional gain by steps of 0.1 until drift becomes acceptable. If after adjusting the proportional gain you still observe a persistent steady-state offset (the drone consistently sits offset from the target by a small, constant amount), add a small integral term by setting POSITION_KI to a value between 0.01 and 0.05. If you need more aggressive damping to prevent overshoot, increase VELOCITY_KP. Test each change thoroughly before making additional adjustments.

## **Testing the System**

Testing the optical position hold system should follow a structured procedure that gradually introduces complexity. Begin with sensor validation, then progress to basic flight testing, and finally evaluate position hold quality.

For sensor validation, start by launching the script and clicking the "Sensor Test (ARM)" button. This connects to the drone and begins streaming sensor data without starting the motors. The GUI will display status messages indicating whether the connection succeeded and whether sensors are providing valid data. Watch the real-time value displays to confirm the height reading is reasonable (should be close to zero when the drone is on the ground), battery voltage is above the LOW_BATTERY_THRESHOLD (typically 2.9V), and velocity values respond when you manually move the drone. If the optical flow sensor is working correctly, you'll see the velocity values change when you slide the drone across a textured surface. Once the sensor test completes successfully and you're confident all sensors are functioning, click "Stop Sensor Test" to prepare for flight.

For basic flight testing, adjust the target height if you want to hover at an altitude different from the default 0.3 meters. Start conservatively with a low height for your first test, perhaps 0.2 or 0.3 meters, until you gain confidence in the system. Make sure the flight area is clear with at least 2 meters of open space in all directions and a textured surface below the drone that provides good optical flow tracking capability. When ready, click "Start Position Hold" to initiate the autonomous takeoff sequence. The drone will rise to the target height, stabilize briefly, then enter the main position hold phase. Watch the trajectory plot in the GUI—the position estimate should remain relatively stable near the origin (marked by the yellow dot), and the current position marker (red dot) should stay close to the target.

To evaluate position hold quality, we need to consider what "good" performance looks like and identify signs of poor tuning or environmental issues. A well-tuned position hold system will keep the drone within approximately ±0.15 meters of the target position when no external disturbances are present. The velocity correction values displayed in the GUI should remain small, typically under 0.05 m/s, indicating that only gentle corrections are needed. The 2D trajectory plot should show the position estimate staying close to the origin with no consistent drift in a particular direction. You may see slow wandering as the dead reckoning estimate accumulates small errors, but this should not consistently favor one direction.

Signs of poor performance include visible oscillation where the drone bounces back and forth repeatedly (indicates position gains are too high), consistent drift in one direction (indicates trim corrections are needed or the surface lacks texture for optical flow tracking), large and frequent correction values (indicates the system is fighting to maintain position, suggesting environmental factors or gain tuning issues), or rapid divergence where the position estimate grows quickly (indicates sensor failure or very poor tracking conditions).

If you observe drift in a consistent direction, you can add trim corrections to compensate. Enter small values (start with ±0.01) in the TRIM VX or TRIM VY fields and click "Apply All Values". The sign of the trim correction should oppose the drift—if the drone drifts forward (positive Y), add a negative TRIM_VY value. Test the correction and adjust iteratively.

The system automatically logs detailed flight data to a CSV file with a timestamp in the filename. After each flight, you can open this file in a spreadsheet application to analyze the position hold performance in detail, examining velocity profiles, position errors, correction commands, and height tracking.

## **Troubleshooting Common Issues**

Several common issues can affect position hold performance. Understanding how to diagnose and resolve these problems will help you achieve reliable operation.

**Position Hold Drift**: If the drone consistently drifts in a particular direction despite position hold being active, several factors could be responsible. First, check the flight surface—the optical flow sensor requires a textured surface with visible patterns to track motion effectively. Plain surfaces, highly reflective surfaces, or transparent surfaces will not work. Second, verify the altitude is within the sensor's operating range. The PMW3901MB optical flow sensor works best between 0.08 and 3 meters altitude. Too low, and the sensor may saturate; too high, and the tracking becomes unreliable. Third, check for external disturbances like air currents from HVAC systems or fans that can overpower the position hold corrections. Fourth, verify your trim corrections (TRIM_VX and TRIM_VY) are appropriate—incorrect trim values can introduce bias that fights against the position controller.

**Oscillation**: If the drone bounces back and forth around the target position, your position controller proportional gain (POSITION_KP) is likely too high. Reduce it by steps of 0.1 until the oscillation stops. You might also increase the velocity controller proportional gain (VELOCITY_KP) to add more damping, which helps suppress oscillation.

**Sensor Data Not Ready**: If the sensor test or flight fails with "Sensor data not ready" messages, this indicates the CRTP logging setup encountered an error. First, verify your drone is running Shield-compatible firmware that includes the motion.deltaX, motion.deltaY, and stateEstimate.z log variables. Second, check that the Flight Stabilizer Shield is properly installed with all connections secure. Third, confirm you can ping the drone's IP address (192.168.43.42 by default) to verify basic network connectivity. Fourth, try restarting the drone and your PC, then run the sensor test again.

**Battery Warnings**: If the system refuses to start flight due to low battery voltage, charge your battery before attempting flight. Flying with low battery voltage risks unexpected power loss during flight and potential crash. The LOW_BATTERY_THRESHOLD is set conservatively to ensure safe operation.

**Large Position Errors**: If the 2D trajectory plot shows the position estimate diverging rapidly from the origin, this indicates poor optical flow tracking. Possible causes include inappropriate flight surface (too smooth or reflective), altitude outside the sensor's operating range, insufficient lighting (the optical flow sensor needs adequate illumination to see surface texture), or actual sensor malfunction. Try moving to a different flight area with better surface texture and lighting conditions.

**Debug Mode**: If you want to test the control algorithms and sensor processing without actually flying the drone, enable DEBUG_MODE by checking the "Debug Mode" checkbox in the GUI. In debug mode, all motor commands are suppressed—the motors will not spin—but sensor logging, position integration, and control calculations proceed normally. This allows you to validate your setup and tune parameters safely while observing the GUI displays and logged data.

## **Conclusion**

The optical position hold system demonstrates the fundamental principles of autonomous flight control using dead reckoning and cascaded PID controllers. By integrating optical flow velocity measurements over time and applying feedback control to maintain the estimated position, the drone achieves stable hover without external position references like GPS. This approach works well in indoor environments and provides an excellent foundation for understanding more advanced autonomous flight capabilities.

The techniques you've learned here—velocity estimation from optical flow, position integration through dead reckoning, drift compensation through thresholding and decay, and cascaded PID control—form the basis for many autonomous navigation systems. As you gain experience with position hold, you can extend these concepts to implement waypoint navigation, obstacle avoidance, and other complex behaviors that build upon the ability to estimate and control position.
