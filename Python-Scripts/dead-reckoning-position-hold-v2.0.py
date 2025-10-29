import time
import threading
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import tkinter as tk
from tkinter import ttk  # Import ttk for better-looking widgets
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
import numpy as np
import csv
import os
from datetime import datetime


# Inline robust CRTP send + NeoPixel helpers (copied/adapted from neopixel_control.py)
def _send_crtp_with_fallback(cf, port, channel, payload: bytes):
    header = ((port & 0x0F) << 4) | (channel & 0x0F)

    class _PacketObj:
        def __init__(self, header, data: bytes):
            self.header = header
            self.data = data
            try:
                self.datat = tuple(data)
            except Exception:
                self.datat = tuple()

        def is_data_size_valid(self):
            return len(self.data) <= 30

        @property
        def size(self):
            return len(self.data)

        def raw(self):
            return bytes([self.header]) + self.data

    pkt_obj = _PacketObj(header, payload)

    # 1) Crazyflie.send_packet if available
    try:
        send_fn = getattr(cf, "send_packet", None)
        if callable(send_fn):
            try:
                send_fn(pkt_obj)
                return
            except Exception:
                pass
    except Exception:
        pass

    # 2) low-level link object: _link or link
    try:
        link = getattr(cf, "_link", None) or getattr(cf, "link", None)
        if link is not None:
            if hasattr(link, "sendPacket"):
                try:
                    link.sendPacket(pkt_obj)
                    return
                except Exception:
                    pass
            if hasattr(link, "send_packet"):
                try:
                    link.send_packet(pkt_obj)
                    return
                except Exception:
                    pass
    except Exception:
        pass

    # 3) cflib.crtp.send_packet fallback (object or raw bytes)
    try:
        import cflib.crtp as _crtp

        sendp = getattr(_crtp, "send_packet", None)
        if callable(sendp):
            try:
                sendp(pkt_obj)
                return
            except Exception:
                try:
                    sendp(bytes([pkt_obj.header]) + pkt_obj.data)
                    return
                except Exception:
                    pass
    except Exception:
        pass

    raise RuntimeError(
        "Unable to send CRTP packet: no send method available on Crazyflie instance"
    )


# NeoPixel utility wrappers that reuse the same Crazyflie link
CRTP_PORT_NEOPIXEL = 0x09
NEOPIXEL_CHANNEL_SET_PIXEL = 0x00
NEOPIXEL_CHANNEL_SHOW = 0x01
NEOPIXEL_CHANNEL_CLEAR = 0x02
NEOPIXEL_CHANNEL_BLINK = 0x03

# NeoPixel send tuning
NP_SEND_RETRIES = 3
NP_PACKET_DELAY = 0.02  # seconds between packets
NP_LINK_SETUP_DELAY = 0.12  # wait after opening link before sending


def np_set_pixel(cf, index, r, g, b):
    _send_crtp_with_fallback(
        cf, CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_SET_PIXEL, bytes([index, r, g, b])
    )


def np_show(cf):
    _send_crtp_with_fallback(cf, CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_SHOW, b"")


def np_clear(cf):
    _send_crtp_with_fallback(cf, CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_CLEAR, b"")


def np_start_blink(cf, on_ms=500, off_ms=500):
    data = bytes(
        [1, (on_ms >> 8) & 0xFF, on_ms & 0xFF, (off_ms >> 8) & 0xFF, off_ms & 0xFF]
    )
    _send_crtp_with_fallback(cf, CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_BLINK, data)


def np_stop_blink(cf):
    data = bytes([0, 0, 0, 0, 0])
    _send_crtp_with_fallback(cf, CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_BLINK, data)


def _try_send_with_retries(cf, fn, *args, retries=NP_SEND_RETRIES):
    """Call a np_* function with retries and small inter-packet delay.
    fn is expected to be a function taking (cf, *args).
    Returns True on success, False on failure.

    Added verbose debug logging to trace which calls are attempted and why
    they may fail. This prints attempt number, function name, args, and any
    exception encountered. Keep the output reasonably terse to avoid
    overwhelming the GUI console.
    """
    last_exc = None
    fn_name = getattr(fn, "__name__", repr(fn))
    for attempt in range(1, retries + 1):
        try:
            # Log what we're about to send
            try:
                args_repr = ", ".join(str(a) for a in args)
            except Exception:
                args_repr = repr(args)
            print(
                f"[NeoPixel] Attempt {attempt}/{retries}: {fn_name}({args_repr}) using cf={type(cf)}"
            )

            fn(cf, *args)

            print(f"[NeoPixel] Success: {fn_name} on attempt {attempt}")
            return True
        except Exception as e:
            last_exc = e
            print(f"[NeoPixel] Attempt {attempt} failed: {e}")
            time.sleep(NP_PACKET_DELAY)
    print(f"[NeoPixel] Failed after {retries} attempts: {last_exc}")
    return False


# === CONFIGURATION PARAMETERS ===
DRONE_URI = "udp://192.168.43.42"
TARGET_HEIGHT = 0.3  # Target hover height in meters
TAKEOFF_TIME = 1.5  # Time to takeoff and stabilize
HOVER_DURATION = 10.0  # How long to hover with position hold
LANDING_TIME = 1  # Time to land
# Debug mode - set to True to disable motors (sensors and logging still work)
DEBUG_MODE = False
# Filtering strength for velocity smoothing (0.0 = no smoothing, 1.0 = max smoothing)
VELOCITY_SMOOTHING_ALPHA = 0.9  # Default: 0.7 (previously hardcoded)
# Basic trim corrections
TRIM_VX = 0.0  # Forward/backward trim correction
TRIM_VY = -0.4  # Left/right trim correction
# === DEAD RECKONING POSITION CONTROL PARAMETERS ===
# PID Controller Parameters
# Start here, then increase gradually
POSITION_KP = 1.2
POSITION_KI = 0.0
POSITION_KD = 0.0
VELOCITY_KP = 1.2
VELOCITY_KI = 0.0
VELOCITY_KD = 0.0
# Control limits
MAX_CORRECTION = 0.095  # Maximum control correction allowed
VELOCITY_THRESHOLD = 0.005  # Consider drone "stationary" below this velocity
DRIFT_COMPENSATION_RATE = 0.002  # Gentle pull toward zero when moving slowly
# Position integration and reset
PERIODIC_RESET_INTERVAL = 30.0  # Reset integrated position every 5 seconds
MAX_POSITION_ERROR = 2.0  # Clamp position error to prevent runaway
# Sensor parameters
SENSOR_PERIOD_MS = 10  # Motion sensor update rate
DT = SENSOR_PERIOD_MS / 1000.0
CONTROL_UPDATE_RATE = 0.02  # 50Hz control loop
# Velocity calculation constants
DEG_TO_RAD = 3.14159 / 180.0
# Optical flow scaling - adjust these to match your sensor/setup
OPTICAL_FLOW_SCALE = (
    3.7  # Empirical scaling factor (adjust based on real vs measured distance)
)
USE_HEIGHT_SCALING = False  # Set to False to disable height dependency
# === GLOBAL VARIABLES ===
# Sensor data
current_height = 0.0
motion_delta_x = 0
motion_delta_y = 0
sensor_data_ready = False
# Log file
log_file = None
log_writer = None
# Battery voltage data
current_battery_voltage = 0.0
battery_data_ready = False
# Velocity tracking
current_vx = 0.0
current_vy = 0.0
velocity_x_history = [0.0, 0.0]
velocity_y_history = [0.0, 0.0]
# Dead reckoning position integration
integrated_position_x = 0.0
integrated_position_y = 0.0
last_integration_time = time.time()
last_reset_time = time.time()
# Control corrections
current_correction_vx = 0.0
current_correction_vy = 0.0
# PID Controller state variables
position_integral_x = 0.0
position_integral_y = 0.0
position_derivative_x = 0.0
position_derivative_y = 0.0
last_position_error_x = 0.0
last_position_error_y = 0.0
velocity_integral_x = 0.0
velocity_integral_y = 0.0
velocity_derivative_x = 0.0
velocity_derivative_y = 0.0
last_velocity_error_x = 0.0
last_velocity_error_y = 0.0
# Flight state
flight_phase = "IDLE"
flight_active = False
sensor_test_active = False  # New variable for sensor test state
scf_instance = None
position_integration_enabled = False
# Data history for plotting
max_history_points = 200
time_history = []
velocity_x_history_plot = []
velocity_y_history_plot = []
position_x_history = []
position_y_history = []
correction_vx_history = []
correction_vy_history = []
height_history = []
# Complete trajectory history (never trimmed)
complete_trajectory_x = []
complete_trajectory_y = []
start_time = None
neo_controller = None


def calculate_velocity(delta_value, altitude):
    """Convert optical flow delta to linear velocity"""
    if altitude <= 0:
        return 0.0
    if USE_HEIGHT_SCALING:
        # Original height-dependent calculation
        velocity_constant = (4.2 * DEG_TO_RAD) / (30.0 * DT)
        velocity = delta_value * altitude * velocity_constant
    else:
        # Simplified calculation without height dependency
        # Using empirical scaling factor
        velocity = delta_value * OPTICAL_FLOW_SCALE * DT
    return velocity


def smooth_velocity(new_velocity, history):
    """Simple 2-point smoothing filter with adjustable alpha"""
    history[1] = history[0]
    history[0] = new_velocity
    alpha = VELOCITY_SMOOTHING_ALPHA  # Use the global variable
    smoothed = (history[0] * alpha) + (history[1] * (1 - alpha))
    if abs(smoothed) < VELOCITY_THRESHOLD:
        smoothed = 0.0
    return smoothed


def integrate_position(vx, vy, dt):
    """Dead reckoning: integrate velocity to position"""
    global integrated_position_x, integrated_position_y
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
    integrated_position_x = max(
        -MAX_POSITION_ERROR, min(MAX_POSITION_ERROR, integrated_position_x)
    )
    integrated_position_y = max(
        -MAX_POSITION_ERROR, min(MAX_POSITION_ERROR, integrated_position_y)
    )


def periodic_position_reset():
    """Reset integrated position every few seconds"""
    global integrated_position_x, integrated_position_y, last_reset_time
    current_time = time.time()
    if current_time - last_reset_time >= PERIODIC_RESET_INTERVAL:
        integrated_position_x = 0.0
        integrated_position_y = 0.0
        last_reset_time = current_time
        return True
    return False


def calculate_position_hold_corrections():
    """Calculate control corrections using PID controllers"""
    global current_correction_vx, current_correction_vy
    global position_integral_x, position_integral_y, position_derivative_x, position_derivative_y
    global last_position_error_x, last_position_error_y
    global velocity_integral_x, velocity_integral_y, velocity_derivative_x, velocity_derivative_y
    global last_velocity_error_x, last_velocity_error_y

    if not sensor_data_ready or current_height <= 0:
        current_correction_vx = 0.0
        current_correction_vy = 0.0
        return 0.0, 0.0

    # Calculate position errors (negative because we want to correct toward zero)
    position_error_x = -integrated_position_x
    position_error_y = -integrated_position_y

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
    # Anti-windup: limit integral term
    position_integral_x = max(-0.1, min(0.1, position_integral_x))
    position_integral_y = max(-0.1, min(0.1, position_integral_y))
    position_i_x = position_integral_x * POSITION_KI
    position_i_y = position_integral_y * POSITION_KI
    # Derivative
    position_derivative_x = (
        position_error_x - last_position_error_x
    ) / CONTROL_UPDATE_RATE
    position_derivative_y = (
        position_error_y - last_position_error_y
    ) / CONTROL_UPDATE_RATE
    position_d_x = position_derivative_x * POSITION_KD
    position_d_y = position_derivative_y * POSITION_KD
    # Store current errors for next iteration
    last_position_error_x = position_error_x
    last_position_error_y = position_error_y

    # Velocity PID Controller
    # Proportional
    velocity_p_x = velocity_error_x * VELOCITY_KP
    velocity_p_y = velocity_error_y * VELOCITY_KP
    # Integral (with anti-windup)
    velocity_integral_x += velocity_error_x * CONTROL_UPDATE_RATE
    velocity_integral_y += velocity_error_y * CONTROL_UPDATE_RATE
    # Anti-windup: limit integral term
    velocity_integral_x = max(-0.05, min(0.05, velocity_integral_x))
    velocity_integral_y = max(-0.05, min(0.05, velocity_integral_y))
    velocity_i_x = velocity_integral_x * VELOCITY_KI
    velocity_i_y = velocity_integral_y * VELOCITY_KI
    # Derivative
    velocity_derivative_x = (
        velocity_error_x - last_velocity_error_x
    ) / CONTROL_UPDATE_RATE
    velocity_derivative_y = (
        velocity_error_y - last_velocity_error_y
    ) / CONTROL_UPDATE_RATE
    velocity_d_x = velocity_derivative_x * VELOCITY_KD
    velocity_d_y = velocity_derivative_y * VELOCITY_KD
    # Store current errors for next iteration
    last_velocity_error_x = velocity_error_x
    last_velocity_error_y = velocity_error_y

    # Combine PID outputs
    position_correction_vx = position_p_x + position_i_x + position_d_x
    position_correction_vy = position_p_y + position_i_y + position_d_y
    velocity_correction_vx = velocity_p_x + velocity_i_x + velocity_d_x
    velocity_correction_vy = velocity_p_y + velocity_i_y + velocity_d_y

    # Combine position and velocity corrections
    total_vx = position_correction_vx + velocity_correction_vx
    total_vy = position_correction_vy + velocity_correction_vy

    # Apply limits
    total_vx = max(-MAX_CORRECTION, min(MAX_CORRECTION, total_vx))
    total_vy = max(-MAX_CORRECTION, min(MAX_CORRECTION, total_vy))

    # Store for GUI display
    current_correction_vx = total_vx
    current_correction_vy = total_vy
    return total_vx, total_vy


def update_history():
    """Update data history for plotting"""
    global start_time
    if start_time is None:
        start_time = time.time()
    current_time = time.time() - start_time
    # Add new data points
    time_history.append(current_time)
    velocity_x_history_plot.append(current_vx)
    velocity_y_history_plot.append(current_vy)
    position_x_history.append(integrated_position_x)
    position_y_history.append(integrated_position_y)
    correction_vx_history.append(current_correction_vx)
    correction_vy_history.append(current_correction_vy)
    height_history.append(current_height)
    # Add to complete trajectory (never trimmed)
    complete_trajectory_x.append(integrated_position_x)
    complete_trajectory_y.append(integrated_position_y)
    # Trim history to max points
    if len(time_history) > max_history_points:
        time_history.pop(0)
        velocity_x_history_plot.pop(0)
        velocity_y_history_plot.pop(0)
        position_x_history.pop(0)
        position_y_history.pop(0)
        correction_vx_history.pop(0)
        correction_vy_history.pop(0)
        height_history.pop(0)


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

    # Debug output every 100 callbacks (reduce console spam)
    if hasattr(motion_callback, "debug_counter"):
        motion_callback.debug_counter += 1
    else:
        motion_callback.debug_counter = 0
    if motion_callback.debug_counter % 100 == 0 and (
        abs(motion_delta_x) > 0 or abs(motion_delta_y) > 0
    ):
        print(
            f"Sensor Debug - Height: {current_height:.3f}m, "
            f"Raw Motion: X={motion_delta_x}, Y={motion_delta_y}, "
            f"Velocities: X={raw_velocity_x:.4f}, Y={raw_velocity_y:.4f}"
        )

    # Apply smoothing
    current_vx = smooth_velocity(raw_velocity_x, velocity_x_history)
    current_vy = smooth_velocity(raw_velocity_y, velocity_y_history)

    # Dead reckoning position integration (only when enabled)
    current_time = time.time()
    dt = current_time - last_integration_time
    if 0.001 <= dt <= 0.1 and position_integration_enabled:
        integrate_position(current_vx, current_vy, dt)
    last_integration_time = current_time

    # Update history for GUI
    update_history()


def battery_callback(timestamp, data, logconf):
    """Battery voltage data callback"""
    global current_battery_voltage, battery_data_ready
    # Get battery voltage
    current_battery_voltage = data.get("pm.vbat", 0.0)
    battery_data_ready = True


def setup_logging(cf):
    """Setup motion sensor and battery voltage logging"""
    log_motion = LogConfig(name="Motion", period_in_ms=SENSOR_PERIOD_MS)
    log_battery = LogConfig(
        name="Battery", period_in_ms=500
    )  # Check battery every 500ms

    try:
        toc = cf.log.toc.toc
        # Setup motion logging
        motion_variables = [
            ("motion.deltaX", "int16_t"),
            ("motion.deltaY", "int16_t"),
            ("stateEstimate.z", "float"),
        ]
        added_motion_vars = []
        for var_name, var_type in motion_variables:
            group, name = var_name.split(".")
            if group in toc and name in toc[group]:
                try:
                    log_motion.add_variable(var_name, var_type)
                    added_motion_vars.append(var_name)
                except Exception as e:
                    print(f"Failed to add motion variable {var_name}: {e}")
            else:
                print(f"Motion variable not found: {var_name}")

        if len(added_motion_vars) < 2:
            print("ERROR: Not enough motion variables found!")
            return None, None

        # Setup battery logging
        battery_variables = [("pm.vbat", "float")]
        added_battery_vars = []
        for var_name, var_type in battery_variables:
            group, name = var_name.split(".")
            if group in toc and name in toc[group]:
                try:
                    log_battery.add_variable(var_name, var_type)
                    added_battery_vars.append(var_name)
                    print(f"Added battery variable: {var_name}")
                except Exception as e:
                    print(f"Failed to add battery variable {var_name}: {e}")
            else:
                print(f"Battery variable not found: {var_name}")

        # Setup callbacks
        log_motion.data_received_cb.add_callback(motion_callback)
        if len(added_battery_vars) > 0:
            log_battery.data_received_cb.add_callback(battery_callback)

        # Add configurations
        cf.log.add_config(log_motion)
        if len(added_battery_vars) > 0:
            cf.log.add_config(log_battery)

        time.sleep(0.5)

        # Validate configurations
        if not log_motion.valid:
            print("ERROR: Motion log configuration invalid!")
            return None, None
        if len(added_battery_vars) > 0 and not log_battery.valid:
            print("WARNING: Battery log configuration invalid!")
            # Continue without battery logging
            log_battery = None

        # Start logging
        log_motion.start()
        if log_battery:
            log_battery.start()

        time.sleep(0.5)
        print(
            f"Logging started - Motion: {len(added_motion_vars)} vars, Battery: {len(added_battery_vars)} vars"
        )
        return log_motion, log_battery

    except Exception as e:
        print(f"Logging setup failed: {e}")
        return None, None


def init_csv_logging():
    """Initialize CSV logging for position and height"""
    global log_file, log_writer
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"drone_flight_log_{timestamp}.csv"
    log_file = open(log_filename, mode="w", newline="")
    log_writer = csv.writer(log_file)
    # Write header
    log_writer.writerow(
        [
            "Timestamp (s)",
            "Integrated Position X (m)",
            "Integrated Position Y (m)",
            "Height (m)",
            "Velocity X (m/s)",
            "Velocity Y (m/s)",
            "Correction VX",
            "Correction VY",
        ]
    )
    print(f"Logging to CSV: {log_filename}")


def log_to_csv():
    """Log current state to CSV if logging is active"""
    global log_writer, start_time
    if log_writer is None or start_time is None:
        return
    elapsed = time.time() - start_time
    log_writer.writerow(
        [
            f"{elapsed:.3f}",
            f"{integrated_position_x:.6f}",
            f"{integrated_position_y:.6f}",
            f"{current_height:.6f}",
            f"{current_vx:.6f}",
            f"{current_vy:.6f}",
            f"{current_correction_vx:.6f}",
            f"{current_correction_vy:.6f}",
        ]
    )


def close_csv_logging():
    """Close CSV log file"""
    global log_file
    if log_file:
        log_file.close()
        log_file = None
        print("CSV log closed.")


class DeadReckoningGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Dead Reckoning Position Hold - Real-Time Monitor")
        self.root.geometry("1400x950")  # Increased height to accommodate new controls

        # Flight control variables
        self.flight_thread = None
        self.flight_running = False
        self.sensor_test_thread = None  # New thread variable for sensor test
        self.sensor_test_running = False  # New flag for sensor test running state

        self.create_ui()
        self.setup_plots()

        # NeoPixel state (lazy connect). We store a Crazyflie instance here
        # and a flag indicating whether this GUI owns the link (so we can
        # close it when appropriate). Reuse global SyncCrazyflie when present.
        self.neo_cf = None
        self._neo_owns_link = False
        self.blinking = False

        # Start animation
        self.anim = animation.FuncAnimation(
            self.fig, self.update_plots, interval=100, cache_frame_data=False
        )

    def create_ui(self):
        """Create the user interface"""
        # Control panel
        control_frame = tk.Frame(self.root)
        control_frame.pack(fill=tk.X, padx=10, pady=5)

        # Flight control buttons
        self.start_button = tk.Button(
            control_frame,
            text="Start Flight",
            command=self.start_flight,
            bg="green",
            fg="white",
            font=("Arial", 12),
        )
        self.start_button.pack(side=tk.LEFT, padx=10)

        # Sensor Test button - New button
        self.sensor_test_button = tk.Button(
            control_frame,
            text="Sensor Test",
            command=self.start_sensor_test,
            bg="lightblue",
            fg="black",
            font=("Arial", 12),
        )
        self.sensor_test_button.pack(side=tk.LEFT, padx=10)
        # Blink NeoPixel button - New button
        self.blink_button = tk.Button(
            control_frame,
            text="Blink LEDs",
            command=self.toggle_blink,
            bg="yellow",
            fg="black",
            font=("Arial", 12),
        )
        self.blink_button.pack(side=tk.LEFT, padx=10)

        # Quick stable color controls
        self.set_static_button = tk.Button(
            control_frame,
            text="Set Static",
            command=self.set_static_mode,
            bg="lightgrey",
            fg="black",
            font=("Arial", 11),
        )
        self.set_static_button.pack(side=tk.LEFT, padx=6)

        self.clear_leds_button = tk.Button(
            control_frame,
            text="Clear LEDs",
            command=self.clear_leds,
            bg="lightgrey",
            fg="black",
            font=("Arial", 11),
        )
        self.clear_leds_button.pack(side=tk.LEFT, padx=6)

        # RGB controls (R, G, B spinboxes) and Set Color button
        rgb_frame = tk.Frame(control_frame)
        rgb_frame.pack(side=tk.LEFT, padx=(6, 0))
        tk.Label(rgb_frame, text="R:").pack(side=tk.LEFT)
        self.rgb_r_var = tk.StringVar(value="255")
        self.rgb_r_spin = tk.Spinbox(
            rgb_frame, from_=0, to=255, width=4, textvariable=self.rgb_r_var
        )
        self.rgb_r_spin.pack(side=tk.LEFT)
        tk.Label(rgb_frame, text="G:").pack(side=tk.LEFT, padx=(6, 0))
        self.rgb_g_var = tk.StringVar(value="255")
        self.rgb_g_spin = tk.Spinbox(
            rgb_frame, from_=0, to=255, width=4, textvariable=self.rgb_g_var
        )
        self.rgb_g_spin.pack(side=tk.LEFT)
        tk.Label(rgb_frame, text="B:").pack(side=tk.LEFT, padx=(6, 0))
        self.rgb_b_var = tk.StringVar(value="255")
        self.rgb_b_spin = tk.Spinbox(
            rgb_frame, from_=0, to=255, width=4, textvariable=self.rgb_b_var
        )
        self.rgb_b_spin.pack(side=tk.LEFT)

        self.set_color_button = tk.Button(
            control_frame,
            text="Set Color",
            command=self.set_color_from_ui,
            bg="lightgrey",
            fg="black",
            font=("Arial", 11),
        )
        self.set_color_button.pack(side=tk.LEFT, padx=6)

        self.stop_button = tk.Button(
            control_frame,
            text="Emergency Stop",
            command=self.emergency_stop,
            bg="red",
            fg="white",
            font=("Arial", 12),
        )
        self.stop_button.pack(side=tk.LEFT, padx=10)

        # Flight status
        self.status_var = tk.StringVar(value="Status: Ready")
        self.status_label = tk.Label(
            control_frame,
            textvariable=self.status_var,
            font=("Arial", 12, "bold"),
            fg="blue",
        )
        self.status_label.pack(side=tk.LEFT, padx=20)

        # Combined frame for Real-time values and Runtime Parameters
        combined_frame = tk.Frame(self.root)
        combined_frame.pack(fill=tk.X, padx=10, pady=5)

        # Real-time values display (Left side)
        values_frame = tk.LabelFrame(
            combined_frame, text="Real-Time Values", padx=10, pady=10
        )
        values_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        # Create value displays in a grid
        self.create_value_displays(values_frame)

        # Runtime Adjustable Parameters (Right side)
        runtime_frame = tk.LabelFrame(
            combined_frame, text="Runtime Adjustable Parameters", padx=10, pady=10
        )
        runtime_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))

        # Create runtime controls
        self.create_runtime_controls(runtime_frame)

        # PID Tuning Controls (Half width)
        pid_frame = tk.LabelFrame(
            self.root, text="PID Tuning Controls", padx=10, pady=10
        )
        pid_frame.pack(fill=tk.X, padx=10, pady=5, side=tk.LEFT, anchor=tk.W)

        # Create PID tuning controls
        self.create_pid_controls(pid_frame)

        # Matplotlib figure
        self.fig = Figure(figsize=(14, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def create_value_displays(self, parent):
        """Create real-time value display widgets"""
        # Row 1: Basic values
        row1 = tk.Frame(parent)
        row1.pack(fill=tk.X, pady=2)
        self.height_var = tk.StringVar(value="Height: 0.000m")
        self.phase_var = tk.StringVar(value="Phase: IDLE")
        self.battery_var = tk.StringVar(value="Battery: 0.00V")
        tk.Label(
            row1, textvariable=self.height_var, font=("Arial", 11, "bold"), fg="blue"
        ).pack(side=tk.LEFT, padx=20)
        tk.Label(
            row1, textvariable=self.phase_var, font=("Arial", 11, "bold"), fg="purple"
        ).pack(side=tk.LEFT, padx=20)
        tk.Label(
            row1, textvariable=self.battery_var, font=("Arial", 11, "bold"), fg="orange"
        ).pack(side=tk.LEFT, padx=20)

        # Row 2: Velocities
        row2 = tk.Frame(parent)
        row2.pack(fill=tk.X, pady=2)
        self.vx_var = tk.StringVar(value="VX: 0.000 m/s")
        self.vy_var = tk.StringVar(value="VY: 0.000 m/s")
        tk.Label(row2, textvariable=self.vx_var, font=("Arial", 11)).pack(
            side=tk.LEFT, padx=20
        )
        tk.Label(row2, textvariable=self.vy_var, font=("Arial", 11)).pack(
            side=tk.LEFT, padx=20
        )

        # Row 3: Integrated positions
        row3 = tk.Frame(parent)
        row3.pack(fill=tk.X, pady=2)
        self.pos_x_var = tk.StringVar(value="Position X: 0.000m")
        self.pos_y_var = tk.StringVar(value="Position Y: 0.000m")
        tk.Label(
            row3,
            textvariable=self.pos_x_var,
            font=("Arial", 11, "bold"),
            fg="darkgreen",
        ).pack(side=tk.LEFT, padx=20)
        tk.Label(
            row3,
            textvariable=self.pos_y_var,
            font=("Arial", 11, "bold"),
            fg="darkgreen",
        ).pack(side=tk.LEFT, padx=20)

        # Row 4: Control corrections
        row4 = tk.Frame(parent)
        row4.pack(fill=tk.X, pady=2)
        self.corr_vx_var = tk.StringVar(value="Correction VX: 0.000")
        self.corr_vy_var = tk.StringVar(value="Correction VY: 0.000")
        tk.Label(
            row4, textvariable=self.corr_vx_var, font=("Arial", 11), fg="red"
        ).pack(side=tk.LEFT, padx=20)
        tk.Label(
            row4, textvariable=self.corr_vy_var, font=("Arial", 11), fg="red"
        ).pack(side=tk.LEFT, padx=20)

    # --- NEW FUNCTION: create_runtime_controls ---
    def create_runtime_controls(self, parent):
        """Create runtime adjustable parameter controls"""
        # Use ttk for better-looking widgets
        # Target Height
        target_height_frame = tk.Frame(parent)
        target_height_frame.pack(fill=tk.X, pady=2)
        tk.Label(target_height_frame, text="Target Height (m):", width=18).pack(
            side=tk.LEFT
        )
        self.target_height_var = tk.StringVar(value=str(TARGET_HEIGHT))
        self.target_height_entry = tk.Entry(
            target_height_frame, textvariable=self.target_height_var, width=8
        )
        self.target_height_entry.pack(side=tk.LEFT, padx=5)

        # Takeoff Time
        takeoff_time_frame = tk.Frame(parent)
        takeoff_time_frame.pack(fill=tk.X, pady=2)
        tk.Label(takeoff_time_frame, text="Takeoff Time (s):", width=18).pack(
            side=tk.LEFT
        )
        self.takeoff_time_var = tk.StringVar(value=str(TAKEOFF_TIME))
        self.takeoff_time_entry = tk.Entry(
            takeoff_time_frame, textvariable=self.takeoff_time_var, width=8
        )
        self.takeoff_time_entry.pack(side=tk.LEFT, padx=5)

        # Hover Duration
        hover_duration_frame = tk.Frame(parent)
        hover_duration_frame.pack(fill=tk.X, pady=2)
        tk.Label(hover_duration_frame, text="Hover Duration (s):", width=18).pack(
            side=tk.LEFT
        )
        self.hover_duration_var = tk.StringVar(value=str(HOVER_DURATION))
        self.hover_duration_entry = tk.Entry(
            hover_duration_frame, textvariable=self.hover_duration_var, width=8
        )
        self.hover_duration_entry.pack(side=tk.LEFT, padx=5)

        # Landing Time
        landing_time_frame = tk.Frame(parent)
        landing_time_frame.pack(fill=tk.X, pady=2)
        tk.Label(landing_time_frame, text="Landing Time (s):", width=18).pack(
            side=tk.LEFT
        )
        self.landing_time_var = tk.StringVar(value=str(LANDING_TIME))
        self.landing_time_entry = tk.Entry(
            landing_time_frame, textvariable=self.landing_time_var, width=8
        )
        self.landing_time_entry.pack(side=tk.LEFT, padx=5)

        # Velocity Smoothing Alpha
        vel_smooth_alpha_frame = tk.Frame(parent)
        vel_smooth_alpha_frame.pack(fill=tk.X, pady=2)
        tk.Label(vel_smooth_alpha_frame, text="Velocity Smoothing α:", width=18).pack(
            side=tk.LEFT
        )
        self.vel_smooth_alpha_var = tk.StringVar(value=str(VELOCITY_SMOOTHING_ALPHA))
        self.vel_smooth_alpha_entry = tk.Entry(
            vel_smooth_alpha_frame, textvariable=self.vel_smooth_alpha_var, width=8
        )
        self.vel_smooth_alpha_entry.pack(side=tk.LEFT, padx=5)

        # Max Correction
        max_corr_frame = tk.Frame(parent)
        max_corr_frame.pack(fill=tk.X, pady=2)
        tk.Label(max_corr_frame, text="Max Correction:", width=18).pack(side=tk.LEFT)
        self.max_corr_var = tk.StringVar(value=str(MAX_CORRECTION))
        self.max_corr_entry = tk.Entry(
            max_corr_frame, textvariable=self.max_corr_var, width=8
        )
        self.max_corr_entry.pack(side=tk.LEFT, padx=5)

        # Apply button for runtime parameters
        apply_runtime_frame = tk.Frame(parent)
        apply_runtime_frame.pack(fill=tk.X, pady=5)
        self.apply_runtime_button = tk.Button(
            apply_runtime_frame,
            text="Apply Runtime Values",
            command=self.apply_runtime_values,
            bg="lightgreen",
            fg="black",
        )
        self.apply_runtime_button.pack(side=tk.LEFT, padx=5)

    # --- END NEW FUNCTION ---

    def toggle_blink(self):
        """Toggle NeoPixel LED blinking on/off"""
        # Use inline np_* helpers; reuse existing SyncCrazyflie when present
        try:
            if not self.blinking:
                # Start blinking: prefer reusing global scf_instance.cf
                cf = None
                if scf_instance is not None:
                    cf = getattr(scf_instance, "cf", None)

                if cf is not None:
                    # Note: We now use np_start_blink without setting white first.
                    # This preserves whatever color was last set when blinking starts.
                    _try_send_with_retries(cf, np_start_blink, 500, 500)
                else:
                    # Open a temporary link just to send the start command
                    tmp_cf = Crazyflie(rw_cache="./cache")
                    try:
                        with SyncCrazyflie(DRONE_URI, cf=tmp_cf) as scf:
                            tmp = getattr(scf, "cf", tmp_cf)
                            time.sleep(NP_LINK_SETUP_DELAY)
                            _try_send_with_retries(tmp, np_start_blink, 500, 500)
                    except Exception as e:
                        raise

                self.blinking = True
                self.blink_button.config(text="Stop Blinking", bg="orange")
                self.status_var.set("Status: LEDs blinking...")
            else:
                # Stop blinking
                cf = None
                if scf_instance is not None:
                    cf = getattr(scf_instance, "cf", None)

                if cf is not None:
                    _try_send_with_retries(cf, np_stop_blink)
                    _try_send_with_retries(cf, np_clear)
                else:
                    tmp_cf = Crazyflie(rw_cache="./cache")
                    try:
                        with SyncCrazyflie(DRONE_URI, cf=tmp_cf) as scf:
                            tmp = getattr(scf, "cf", tmp_cf)
                            time.sleep(NP_LINK_SETUP_DELAY)
                            _try_send_with_retries(tmp, np_stop_blink)
                            _try_send_with_retries(tmp, np_clear)
                    except Exception:
                        pass

                self.blinking = False
                self.blink_button.config(text="Blink LEDs", bg="yellow")
                self.status_var.set("Status: LEDs off")
        except Exception as e:
            self.status_var.set(f"Status: NeoPixel error - {str(e)}")
            print(f"NeoPixel error: {e}")

    def set_static_mode(self):
        """Make the current LED color static (stop blinking but keep color)."""
        try:
            if self.blinking:
                # reuse existing link or temporary
                cf = (
                    getattr(scf_instance, "cf", None)
                    if scf_instance is not None
                    else None
                )
                if cf is not None:
                    _try_send_with_retries(cf, np_stop_blink)
                else:
                    tmp_cf = Crazyflie(rw_cache="./cache")
                    try:
                        with SyncCrazyflie(DRONE_URI, cf=tmp_cf) as scf:
                            tmp = getattr(scf, "cf", tmp_cf)
                            time.sleep(NP_LINK_SETUP_DELAY)
                            _try_send_with_retries(tmp, np_stop_blink)
                    except Exception as e:
                        print(f"NeoPixel error setting static: {e}")
                        return

                self.blinking = False
                self.blink_button.config(text="Blink LEDs", bg="yellow")
                self.status_var.set("Status: LEDs set to static mode")
        except Exception as e:
            self.status_var.set(f"Status: NeoPixel error - {str(e)}")
            print(f"NeoPixel error: {e}")

    def set_leds_color(self, r, g, b):
        """Set a stable color on the NeoPixels (stops blinking first)."""
        try:
            # If currently blinking, stop it first
            if self.blinking:
                # reuse existing link or temporary
                cf = (
                    getattr(scf_instance, "cf", None)
                    if scf_instance is not None
                    else None
                )
                if cf is not None:
                    np_stop_blink(cf)
                else:
                    tmp_cf = Crazyflie(rw_cache="./cache")
                    try:
                        with SyncCrazyflie(DRONE_URI, cf=tmp_cf) as scf:
                            tmp = getattr(scf, "cf", tmp_cf)
                            np_stop_blink(tmp)
                    except Exception:
                        pass
                self.blinking = False
                self.blink_button.config(text="Blink LEDs", bg="yellow")

            cf = getattr(scf_instance, "cf", None) if scf_instance is not None else None
            if cf is not None:
                for i in range(4):
                    _try_send_with_retries(cf, np_set_pixel, i, r, g, b)
                    time.sleep(NP_PACKET_DELAY)
                _try_send_with_retries(cf, np_show)
            else:
                tmp_cf = Crazyflie(rw_cache="./cache")
                try:
                    with SyncCrazyflie(DRONE_URI, cf=tmp_cf) as scf:
                        tmp = getattr(scf, "cf", tmp_cf)
                        time.sleep(NP_LINK_SETUP_DELAY)
                        for i in range(4):
                            _try_send_with_retries(tmp, np_set_pixel, i, r, g, b)
                            time.sleep(NP_PACKET_DELAY)
                        _try_send_with_retries(tmp, np_show)
                except Exception as e:
                    self.status_var.set(f"Status: NeoPixel error - {str(e)}")
                    print(f"NeoPixel error: {e}")
                    return

            self.status_var.set("Status: LEDs set to color")
        except Exception as e:
            self.status_var.set(f"Status: NeoPixel error - {str(e)}")
            print(f"NeoPixel set color error: {e}")

    def set_color_from_ui(self):
        """Read R,G,B from UI controls and set LEDs accordingly."""
        try:
            r = int(self.rgb_r_var.get())
            g = int(self.rgb_g_var.get())
            b = int(self.rgb_b_var.get())
        except Exception:
            self.status_var.set("Status: Invalid RGB values")
            return

        # Clamp values
        r = max(0, min(255, r))
        g = max(0, min(255, g))
        b = max(0, min(255, b))

        self.set_leds_color(r, g, b)

    def clear_leds(self):
        """Clear all NeoPixels (set to off)."""
        try:
            cf = getattr(scf_instance, "cf", None) if scf_instance is not None else None
            if cf is not None:
                _try_send_with_retries(cf, np_stop_blink)
                _try_send_with_retries(cf, np_clear)
            else:
                tmp_cf = Crazyflie(rw_cache="./cache")
                try:
                    with SyncCrazyflie(DRONE_URI, cf=tmp_cf) as scf:
                        tmp = getattr(scf, "cf", tmp_cf)
                        time.sleep(NP_LINK_SETUP_DELAY)
                        _try_send_with_retries(tmp, np_stop_blink)
                        _try_send_with_retries(tmp, np_clear)
                except Exception:
                    pass

            self.blinking = False
            self.blink_button.config(text="Blink LEDs", bg="yellow")
            self.status_var.set("Status: LEDs cleared")
        except Exception as e:
            self.status_var.set(f"Status: NeoPixel error - {str(e)}")
            print(f"NeoPixel clear error: {e}")

    def create_pid_controls(self, parent):
        """Create PID tuning input controls with TRIM controls - compact layout"""
        # Main control frame - horizontal layout for PID and TRIM sections
        main_control_frame = tk.Frame(parent)
        main_control_frame.pack(fill=tk.X, pady=2)

        # Left side - PID Controls
        pid_frame = tk.LabelFrame(
            main_control_frame, text="PID Controls", padx=5, pady=5
        )
        pid_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        # Position PID Controls (horizontal layout)
        pos_frame = tk.LabelFrame(pid_frame, text="Position PID", padx=5, pady=2)
        pos_frame.pack(fill=tk.X, pady=2)
        pos_row = tk.Frame(pos_frame)
        pos_row.pack(fill=tk.X)
        # Position Kp
        tk.Label(pos_row, text="Kp:", width=3).pack(side=tk.LEFT)
        self.pos_kp_var = tk.StringVar(value=str(POSITION_KP))
        self.pos_kp_entry = tk.Entry(pos_row, textvariable=self.pos_kp_var, width=6)
        self.pos_kp_entry.pack(side=tk.LEFT, padx=2)
        # Position Ki
        tk.Label(pos_row, text="Ki:", width=3).pack(side=tk.LEFT, padx=(5, 0))
        self.pos_ki_var = tk.StringVar(value=str(POSITION_KI))
        self.pos_ki_entry = tk.Entry(pos_row, textvariable=self.pos_ki_var, width=6)
        self.pos_ki_entry.pack(side=tk.LEFT, padx=2)
        # Position Kd
        tk.Label(pos_row, text="Kd:", width=3).pack(side=tk.LEFT, padx=(5, 0))
        self.pos_kd_var = tk.StringVar(value=str(POSITION_KD))
        self.pos_kd_entry = tk.Entry(pos_row, textvariable=self.pos_kd_var, width=6)
        self.pos_kd_entry.pack(side=tk.LEFT, padx=2)

        # Velocity PID Controls (horizontal layout)
        vel_frame = tk.LabelFrame(pid_frame, text="Velocity PID", padx=5, pady=2)
        vel_frame.pack(fill=tk.X, pady=2)
        vel_row = tk.Frame(vel_frame)
        vel_row.pack(fill=tk.X)
        # Velocity Kp
        tk.Label(vel_row, text="Kp:", width=3).pack(side=tk.LEFT)
        self.vel_kp_var = tk.StringVar(value=str(VELOCITY_KP))
        self.vel_kp_entry = tk.Entry(vel_row, textvariable=self.vel_kp_var, width=6)
        self.vel_kp_entry.pack(side=tk.LEFT, padx=2)
        # Velocity Ki
        tk.Label(vel_row, text="Ki:", width=3).pack(side=tk.LEFT, padx=(5, 0))
        self.vel_ki_var = tk.StringVar(value=str(VELOCITY_KI))
        self.vel_ki_entry = tk.Entry(vel_row, textvariable=self.vel_ki_var, width=6)
        self.vel_ki_entry.pack(side=tk.LEFT, padx=2)
        # Velocity Kd
        tk.Label(vel_row, text="Kd:", width=3).pack(side=tk.LEFT, padx=(5, 0))
        self.vel_kd_var = tk.StringVar(value=str(VELOCITY_KD))
        self.vel_kd_entry = tk.Entry(vel_row, textvariable=self.vel_kd_var, width=6)
        self.vel_kd_entry.pack(side=tk.LEFT, padx=2)

        # Right side - TRIM Controls
        trim_frame = tk.LabelFrame(
            main_control_frame, text="TRIM Controls", padx=5, pady=5
        )
        trim_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(10, 0))

        # TRIM VX Control
        trim_vx_frame = tk.Frame(trim_frame)
        trim_vx_frame.pack(fill=tk.X, pady=5)
        tk.Label(trim_vx_frame, text="TRIM VX:", width=10).pack(side=tk.LEFT)
        self.trim_vx_var = tk.StringVar(value=str(TRIM_VX))
        self.trim_vx_entry = tk.Entry(
            trim_vx_frame, textvariable=self.trim_vx_var, width=8
        )
        self.trim_vx_entry.pack(side=tk.LEFT, padx=2)

        # TRIM VY Control
        trim_vy_frame = tk.Frame(trim_frame)
        trim_vy_frame.pack(fill=tk.X, pady=5)
        tk.Label(trim_vy_frame, text="TRIM VY:", width=10).pack(side=tk.LEFT)
        self.trim_vy_var = tk.StringVar(value=str(TRIM_VY))
        self.trim_vy_entry = tk.Entry(
            trim_vy_frame, textvariable=self.trim_vy_var, width=8
        )
        self.trim_vy_entry.pack(side=tk.LEFT, padx=2)

        # Helper labels for TRIM values
        trim_help_frame = tk.Frame(trim_frame)
        trim_help_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            trim_help_frame,
            text="(Forward/Back, Left/Right)",
            font=("Arial", 8),
            fg="gray",
        ).pack()

        # Optical Flow Scaling Controls
        scale_frame = tk.LabelFrame(
            main_control_frame, text="Optical Flow Scaling", padx=5, pady=5
        )
        scale_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(10, 0))

        # Scaling factor control
        scale_factor_frame = tk.Frame(scale_frame)
        scale_factor_frame.pack(fill=tk.X, pady=5)
        tk.Label(scale_factor_frame, text="Scale Factor:", width=12).pack(side=tk.LEFT)
        self.scale_factor_var = tk.StringVar(value=str(OPTICAL_FLOW_SCALE))
        self.scale_factor_entry = tk.Entry(
            scale_factor_frame, textvariable=self.scale_factor_var, width=8
        )
        self.scale_factor_entry.pack(side=tk.LEFT, padx=2)

        # Height scaling checkbox
        height_scale_frame = tk.Frame(scale_frame)
        height_scale_frame.pack(fill=tk.X, pady=5)
        self.height_scaling_var = tk.BooleanVar(value=USE_HEIGHT_SCALING)
        self.height_scaling_check = tk.Checkbutton(
            height_scale_frame,
            text="Use Height Scaling",
            variable=self.height_scaling_var,
        )
        self.height_scaling_check.pack()

        # Helper text for scaling
        scale_help_frame = tk.Frame(scale_frame)
        scale_help_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            scale_help_frame,
            text="(Increase if trajectory too small)",
            font=("Arial", 8),
            fg="gray",
        ).pack()

        # Control buttons (horizontal layout) - span across both sections
        button_frame = tk.Frame(parent)
        button_frame.pack(fill=tk.X, pady=5)

        self.apply_all_button = tk.Button(
            button_frame,
            text="Apply All Values",
            command=self.apply_all_values,
            bg="green",
            fg="black",
        )
        self.apply_all_button.pack(side=tk.LEFT, padx=5)

        self.reset_all_button = tk.Button(
            button_frame,
            text="Reset to Default",
            command=self.reset_all_values,
            bg="orange",
            fg="black",
        )
        self.reset_all_button.pack(side=tk.LEFT, padx=5)

        self.clear_graphs_button = tk.Button(
            button_frame,
            text="Clear Graphs",
            command=self.clear_graphs,
            bg="blue",
            fg="black",
        )
        self.clear_graphs_button.pack(side=tk.LEFT, padx=5)

    def setup_plots(self):
        """Setup matplotlib plots"""
        # Create 2x2 subplot layout
        self.ax1 = self.fig.add_subplot(2, 2, 1)  # Velocities
        self.ax2 = self.fig.add_subplot(2, 2, 2)  # Integrated Position (2D plot)
        self.ax3 = self.fig.add_subplot(2, 2, 3)  # Control Corrections
        self.ax4 = self.fig.add_subplot(2, 2, 4)  # Height

        # Velocities plot
        self.ax1.set_title("Velocities (m/s)", fontsize=12)
        self.ax1.set_ylabel("Velocity (m/s)")
        self.ax1.grid(True, alpha=0.3)
        (self.line_vx,) = self.ax1.plot([], [], "b-", linewidth=2, label="VX")
        (self.line_vy,) = self.ax1.plot([], [], "r-", linewidth=2, label="VY")
        self.ax1.legend()

        # 2D Position plot
        self.ax2.set_title("Integrated Position (Dead Reckoning)", fontsize=12)
        self.ax2.set_xlabel("X Position (m)")
        self.ax2.set_ylabel("Y Position (m)")
        self.ax2.set_aspect("equal")
        self.ax2.grid(True, alpha=0.3)
        (self.line_pos,) = self.ax2.plot(
            [], [], "purple", linewidth=2, alpha=0.7, label="Trajectory"
        )
        (self.current_pos,) = self.ax2.plot([], [], "ro", markersize=8, label="Current")
        self.ax2.plot(
            0,
            0,
            "ko",
            markersize=10,
            markerfacecolor="yellow",
            markeredgecolor="black",
            label="Origin",
        )
        self.ax2.legend()

        # Control corrections plot
        self.ax3.set_title("Control Corrections", fontsize=12)
        self.ax3.set_ylabel("Correction")
        self.ax3.grid(True, alpha=0.3)
        (self.line_corr_vx,) = self.ax3.plot([], [], "g-", linewidth=2, label="Corr VX")
        (self.line_corr_vy,) = self.ax3.plot([], [], "m-", linewidth=2, label="Corr VY")
        self.ax3.legend()

        # Height plot
        self.ax4.set_title("Height", fontsize=12)
        self.ax4.set_xlabel("Time (s)")
        self.ax4.set_ylabel("Height (m)")
        self.ax4.grid(True, alpha=0.3)
        (self.line_height,) = self.ax4.plot(
            [], [], "orange", linewidth=2, label="Height"
        )
        self.ax4.axhline(
            y=TARGET_HEIGHT, color="red", linestyle="--", alpha=0.7, label="Target"
        )
        self.ax4.legend()

        self.fig.tight_layout()

    def update_plots(self, frame):
        """Update all plots with new data"""
        if not time_history:
            return []

        # Update real-time value displays
        self.height_var.set(f"Height: {current_height:.3f}m")
        self.phase_var.set(f"Phase: {flight_phase}")
        # Update battery voltage with color coding
        if current_battery_voltage > 0:
            if current_battery_voltage < 3.4:
                battery_color = "red"
                battery_status = " (LOW!)"
            elif current_battery_voltage < 3.5:
                battery_color = "orange"
                battery_status = " (Warning)"
            else:
                battery_color = "green"
                battery_status = ""
            self.battery_var.set(
                f"Battery: {current_battery_voltage:.2f}V{battery_status}"
            )
        else:
            self.battery_var.set("Battery: N/A")
        self.vx_var.set(f"VX: {current_vx:.3f} m/s")
        self.vy_var.set(f"VY: {current_vy:.3f} m/s")
        self.pos_x_var.set(f"Position X: {integrated_position_x:.3f}m")
        self.pos_y_var.set(f"Position Y: {integrated_position_y:.3f}m")
        self.corr_vx_var.set(f"Correction VX: {current_correction_vx:.3f}")
        self.corr_vy_var.set(f"Correction VY: {current_correction_vy:.3f}")

        # Update plots
        try:
            # Velocities
            self.line_vx.set_data(time_history, velocity_x_history_plot)
            self.line_vy.set_data(time_history, velocity_y_history_plot)

            # 2D Position - use complete trajectory (never trimmed)
            if complete_trajectory_x and complete_trajectory_y:
                # Fix coordinate system: negate X for correct visualization
                plot_x = [-x for x in complete_trajectory_x]
                self.line_pos.set_data(plot_x, complete_trajectory_y)
                self.current_pos.set_data(
                    [-integrated_position_x], [integrated_position_y]
                )

            # Control corrections
            self.line_corr_vx.set_data(time_history, correction_vx_history)
            self.line_corr_vy.set_data(time_history, correction_vy_history)

            # Height
            self.line_height.set_data(time_history, height_history)

            # Adjust axis limits
            if len(time_history) > 1:
                time_range = max(time_history) - min(time_history)
                time_margin = time_range * 0.05
                # Time-based plots
                for ax in [self.ax1, self.ax3, self.ax4]:
                    ax.set_xlim(
                        min(time_history) - time_margin, max(time_history) + time_margin
                    )

                # Velocity plot
                if velocity_x_history_plot and velocity_y_history_plot:
                    all_vel = velocity_x_history_plot + velocity_y_history_plot
                    if any(v != 0 for v in all_vel):
                        vel_range = max(all_vel) - min(all_vel)
                        vel_margin = max(vel_range * 0.1, 0.01)
                        self.ax1.set_ylim(
                            min(all_vel) - vel_margin, max(all_vel) + vel_margin
                        )

                # Position plot - use complete trajectory for axis limits
                if complete_trajectory_x and complete_trajectory_y:
                    # Fix coordinate system for axis limits too
                    plot_x = [-x for x in complete_trajectory_x]
                    pos_range_x = max(plot_x) - min(plot_x)
                    pos_range_y = max(complete_trajectory_y) - min(
                        complete_trajectory_y
                    )
                    max_range = max(pos_range_x, pos_range_y, 0.02)  # Minimum range
                    center_x = (max(plot_x) + min(plot_x)) / 2
                    center_y = (
                        max(complete_trajectory_y) + min(complete_trajectory_y)
                    ) / 2
                    margin = max_range * 0.6
                    self.ax2.set_xlim(center_x - margin, center_x + margin)
                    self.ax2.set_ylim(center_y - margin, center_y + margin)

                # Control corrections
                if correction_vx_history and correction_vy_history:
                    all_corr = correction_vx_history + correction_vy_history
                    if any(c != 0 for c in all_corr):
                        corr_range = max(all_corr) - min(all_corr)
                        corr_margin = max(corr_range * 0.1, 0.01)
                        self.ax3.set_ylim(
                            min(all_corr) - corr_margin, max(all_corr) + corr_margin
                        )

                # Height plot
                if height_history:
                    height_range = max(height_history) - min(height_history)
                    height_margin = max(height_range * 0.1, 0.05)
                    self.ax4.set_ylim(
                        min(height_history) - height_margin,
                        max(height_history) + height_margin,
                    )

                # Update the target height line on the height plot
                self.ax4.collections = []  # Clear previous target line
                self.ax4.axhline(
                    y=float(self.target_height_var.get()),
                    color="red",
                    linestyle="--",
                    alpha=0.7,
                    label="Target",
                )

        except Exception as e:
            pass  # Ignore plotting errors

        return []

    # --- NEW FUNCTION: apply_runtime_values ---
    def apply_runtime_values(self):
        """Apply runtime adjustable values from GUI inputs"""
        global TARGET_HEIGHT, TAKEOFF_TIME, HOVER_DURATION, LANDING_TIME, VELOCITY_SMOOTHING_ALPHA, MAX_CORRECTION
        try:
            # Get values from GUI
            new_target_height = float(self.target_height_var.get())
            new_takeoff_time = float(self.takeoff_time_var.get())
            new_hover_duration = float(self.hover_duration_var.get())
            new_landing_time = float(self.landing_time_var.get())
            new_vel_smooth_alpha = float(self.vel_smooth_alpha_var.get())
            new_max_corr = float(self.max_corr_var.get())

            # Validate values (optional, add checks as needed)
            if new_takeoff_time < 0 or new_hover_duration < 0 or new_landing_time < 0:
                raise ValueError("Time values cannot be negative.")
            if new_vel_smooth_alpha < 0 or new_vel_smooth_alpha > 1.0:
                raise ValueError("Smoothing alpha must be between 0 and 1.")
            if new_max_corr < 0:
                raise ValueError("Max correction cannot be negative.")

            # Apply values to global variables
            TARGET_HEIGHT = new_target_height
            TAKEOFF_TIME = new_takeoff_time
            HOVER_DURATION = new_hover_duration
            LANDING_TIME = new_landing_time
            VELOCITY_SMOOTHING_ALPHA = new_vel_smooth_alpha
            MAX_CORRECTION = new_max_corr

            print(f"Runtime Values Applied:")
            print(f"  Target Height: {TARGET_HEIGHT}")
            print(f"  Takeoff Time: {TAKEOFF_TIME}")
            print(f"  Hover Duration: {HOVER_DURATION}")
            print(f"  Landing Time: {LANDING_TIME}")
            print(f"  Velocity Smoothing Alpha: {VELOCITY_SMOOTHING_ALPHA}")
            print(f"  Max Correction: {MAX_CORRECTION}")

        except ValueError as e:
            print(f"Error applying runtime values: {e}")
            print("Please enter valid numbers for all runtime parameters.")

    # --- END NEW FUNCTION ---

    def apply_pid_values(self):
        """Apply PID values from GUI inputs"""
        global POSITION_KP, POSITION_KI, POSITION_KD, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD
        try:
            # Get values from GUI
            POSITION_KP = float(self.pos_kp_var.get())
            POSITION_KI = float(self.pos_ki_var.get())
            POSITION_KD = float(self.pos_kd_var.get())
            VELOCITY_KP = float(self.vel_kp_var.get())
            VELOCITY_KI = float(self.vel_ki_var.get())
            VELOCITY_KD = float(self.vel_kd_var.get())

            # Reset PID state when applying new values
            global position_integral_x, position_integral_y, velocity_integral_x, velocity_integral_y
            global last_position_error_x, last_position_error_y, last_velocity_error_x, last_velocity_error_y
            position_integral_x = 0.0
            position_integral_y = 0.0
            velocity_integral_x = 0.0
            velocity_integral_y = 0.0
            last_position_error_x = 0.0
            last_position_error_y = 0.0
            last_velocity_error_x = 0.0
            last_velocity_error_y = 0.0

            print(f"PID Values Applied:")
            print(f"Position: Kp={POSITION_KP}, Ki={POSITION_KI}, Kd={POSITION_KD}")
            print(f"Velocity: Kp={VELOCITY_KP}, Ki={VELOCITY_KI}, Kd={VELOCITY_KD}")
        except ValueError as e:
            print(f"Error applying PID values: {e}")
            print("Please enter valid numbers")

    def reset_pid_values(self):
        """Reset PID values to default"""
        # Default values (from your current settings)
        self.pos_kp_var.set("1.2")
        self.pos_ki_var.set("0.00")
        self.pos_kd_var.set("0.0")
        self.vel_kp_var.set("1.2")
        self.vel_ki_var.set("0.0")
        self.vel_kd_var.set("0.0")
        # Apply the reset values
        self.apply_pid_values()
        print("PID values reset to default")

    def apply_all_values(self):
        """Apply PID, TRIM, and Optical Flow scaling values from GUI inputs"""
        # First apply PID values
        self.apply_pid_values()
        # Then apply TRIM values
        global TRIM_VX, TRIM_VY, OPTICAL_FLOW_SCALE, USE_HEIGHT_SCALING
        try:
            TRIM_VX = float(self.trim_vx_var.get())
            TRIM_VY = float(self.trim_vy_var.get())
            print(f"TRIM Values Applied: VX={TRIM_VX}, VY={TRIM_VY}")
        except ValueError as e:
            print(f"Error applying TRIM values: {e}")
            print("Please enter valid numbers for TRIM values")

        # Apply Optical Flow scaling values
        try:
            OPTICAL_FLOW_SCALE = float(self.scale_factor_var.get())
            USE_HEIGHT_SCALING = self.height_scaling_var.get()
            print(
                f"Optical Flow Scaling Applied: Scale={OPTICAL_FLOW_SCALE}, Height Scaling={USE_HEIGHT_SCALING}"
            )
        except ValueError as e:
            print(f"Error applying Optical Flow scaling: {e}")
            print("Please enter valid numbers for scaling factor")

    def reset_all_values(self):
        """Reset PID, TRIM, and Optical Flow scaling values to default"""
        # Reset PID values
        self.reset_pid_values()
        # Reset TRIM values to current defaults
        self.trim_vx_var.set("0.1")
        self.trim_vy_var.set("-0.02")
        # Reset Optical Flow scaling values
        self.scale_factor_var.set("3.7")
        self.height_scaling_var.set(False)
        # Reset Runtime values to defaults
        self.target_height_var.set("0.2")
        self.takeoff_time_var.set("1.0")
        self.hover_duration_var.set("30.0")
        self.landing_time_var.set("0.5")
        self.vel_smooth_alpha_var.set("0.8")
        self.max_corr_var.set("0.1")
        # Apply all values
        global TRIM_VX, TRIM_VY, OPTICAL_FLOW_SCALE, USE_HEIGHT_SCALING
        TRIM_VX = 0.1
        TRIM_VY = -0.02
        OPTICAL_FLOW_SCALE = 3.7
        USE_HEIGHT_SCALING = False
        print("All values reset to default")

    def clear_graphs(self):
        """Clear all graph data and reset plotting"""
        global time_history, velocity_x_history_plot, velocity_y_history_plot
        global position_x_history, position_y_history, correction_vx_history, correction_vy_history
        global height_history, complete_trajectory_x, complete_trajectory_y, start_time
        # Clear all history arrays
        time_history.clear()
        velocity_x_history_plot.clear()
        velocity_y_history_plot.clear()
        position_x_history.clear()
        position_y_history.clear()
        correction_vx_history.clear()
        correction_vy_history.clear()
        height_history.clear()
        complete_trajectory_x.clear()
        complete_trajectory_y.clear()
        # Reset start time
        start_time = None
        # Clear plot lines
        self.line_vx.set_data([], [])
        self.line_vy.set_data([], [])
        self.line_pos.set_data([], [])
        self.current_pos.set_data([], [])
        self.line_corr_vx.set_data([], [])
        self.line_corr_vy.set_data([], [])
        self.line_height.set_data([], [])
        # Reset plot axes to default ranges
        self.ax1.set_xlim(0, 10)
        self.ax1.set_ylim(-0.1, 0.1)
        self.ax2.set_xlim(-0.05, 0.05)
        self.ax2.set_ylim(-0.05, 0.05)
        self.ax3.set_xlim(0, 10)
        self.ax3.set_ylim(-0.1, 0.1)
        self.ax4.set_xlim(0, 10)
        self.ax4.set_ylim(0.2, 0.4)
        print("All graphs cleared")

    def start_sensor_test(self):  # New function for sensor test
        """Start the sensor test in a separate thread"""
        if (
            not self.sensor_test_running and not self.flight_running
        ):  # Prevent starting if flight is active
            self.sensor_test_running = True
            self.sensor_test_button.config(state=tk.DISABLED, text="Sensor Testing...")
            self.status_var.set("Status: Starting Sensor Test...")
            self.sensor_test_thread = threading.Thread(
                target=self.sensor_test_controller_thread
            )
            self.sensor_test_thread.daemon = True
            self.sensor_test_thread.start()
        elif self.flight_running:
            print("Cannot start Sensor Test while Flight is active.")
            self.status_var.set("Status: Flight Active - Cannot Test Sensors")

    def sensor_test_controller_thread(self):  # New thread function for sensor test
        """Sensor test controller running in separate thread"""
        global flight_phase, sensor_test_active, scf_instance
        global integrated_position_x, integrated_position_y, last_integration_time, last_reset_time
        global position_integration_enabled  # Need to access this to enable integration

        sensor_test_active = True
        flight_phase = "SENSOR_TEST"  # Update phase

        cflib.crtp.init_drivers()
        cf = Crazyflie(rw_cache="./cache")
        log_motion = None
        log_battery = None

        try:
            with SyncCrazyflie(DRONE_URI, cf=cf) as scf:
                scf_instance = scf
                # Setup logging (same as flight)
                log_motion, log_battery = setup_logging(cf)
                use_position_hold = log_motion is not None
                if use_position_hold:
                    time.sleep(1.0)

                # Initialize flight parameters (skip if in debug mode, but logging still happens)
                if not DEBUG_MODE:
                    cf.commander.send_setpoint(0, 0, 0, 0)
                    time.sleep(0.1)
                    cf.param.set_value("commander.enHighLevel", "1")
                    time.sleep(0.5)
                else:
                    print("DEBUG MODE: Skipping flight initialization for sensor test")

                # Enable position integration for sensor test
                integrated_position_x = 0.0
                integrated_position_y = 0.0
                last_integration_time = time.time()
                last_reset_time = time.time()
                position_integration_enabled = True  # Enable position integration

                # Reset PID controller state (not used in sensor test, but good practice)
                global position_integral_x, position_integral_y, velocity_integral_x, velocity_integral_y
                global last_position_error_x, last_position_error_y, last_velocity_error_x, last_velocity_error_y
                position_integral_x = 0.0
                position_integral_y = 0.0
                velocity_integral_x = 0.0
                velocity_integral_y = 0.0
                last_position_error_x = 0.0
                last_position_error_y = 0.0
                last_velocity_error_x = 0.0
                last_velocity_error_y = 0.0

                # Run sensor test loop (no motor commands)
                flight_phase = "SENSOR_TEST"
                start_time = time.time()
                init_csv_logging()
                while sensor_test_active:  # Continue while sensor test is active
                    # Calculate corrections (they will be 0 if PID params are 0, but still updates internal state)
                    if use_position_hold and sensor_data_ready:
                        motion_vx, motion_vy = calculate_position_hold_corrections()
                        # Check for periodic reset
                        if periodic_position_reset():
                            flight_phase = "SENSOR_TEST (RESET)"
                        else:
                            flight_phase = "SENSOR_TEST"
                    time.sleep(CONTROL_UPDATE_RATE)  # Maintain control loop rate
                    log_to_csv()
        except Exception as e:
            flight_phase = f"ERROR: {str(e)}"
        finally:
            # Stop logging
            close_csv_logging()
            if log_motion:
                try:
                    log_motion.stop()
                except:
                    pass
            if log_battery:
                try:
                    log_battery.stop()
                except:
                    pass
            sensor_test_active = False
            flight_phase = "IDLE"
            self.sensor_test_running = False
            self.sensor_test_button.config(state=tk.NORMAL, text="Sensor Test")
            self.status_var.set("Status: Sensor Test Stopped")

    def start_flight(self):
        """Start the flight in a separate thread with battery and sensor safety checks"""
        if not self.flight_running and not self.sensor_test_running:
            # Battery safety check
            if current_battery_voltage > 0 and current_battery_voltage < 3.4:
                self.status_var.set(
                    f"Status: Battery too low ({current_battery_voltage:.2f}V)! Cannot start flight."
                )
                print(
                    f"SAFETY: Flight blocked - Battery voltage {current_battery_voltage:.2f}V is below 3.5V minimum"
                )
                return
            elif current_battery_voltage == 0.0:
                print("WARNING: Battery voltage unknown")

            # SENSOR SAFETY CHECK: ensure height and motion data are flowing
            if not sensor_data_ready:
                self.status_var.set(
                    "Status: Sensor data not ready! Wait for height & motion readings."
                )
                print("SAFETY: Flight blocked - No sensor data received yet.")
                return

            if current_height <= 0.0:  # e.g., drone on ground or invalid
                self.status_var.set(
                    "Status: Invalid height reading! Ensure drone is powered and sensors active."
                )
                print(
                    "SAFETY: Flight blocked - Height too low or invalid:",
                    current_height,
                )
                return

            # Proceed
            self.flight_running = True
            self.start_button.config(state=tk.DISABLED)
            self.status_var.set("Status: Starting Flight...")
            self.flight_thread = threading.Thread(target=self.flight_controller_thread)
            self.flight_thread.daemon = True
            self.flight_thread.start()
        elif self.sensor_test_running:
            print("Cannot start Flight while Sensor Test is active.")
            self.status_var.set("Status: Sensor Test Active - Cannot Start Flight")

    def emergency_stop(self):
        """Emergency stop the flight or sensor test"""
        global flight_active, sensor_test_active
        flight_active = False
        sensor_test_active = False  # Stop sensor test as well
        self.flight_running = False
        self.sensor_test_running = False  # Reset sensor test flag
        self.start_button.config(state=tk.NORMAL)
        self.sensor_test_button.config(
            state=tk.NORMAL, text="Sensor Test"
        )  # Reset button text
        self.status_var.set("Status: Emergency Stopped")

    def flight_controller_thread(self):
        """Flight controller running in separate thread"""
        global flight_phase, flight_active, scf_instance
        global integrated_position_x, integrated_position_y, last_integration_time, last_reset_time

        cflib.crtp.init_drivers()
        cf = Crazyflie(rw_cache="./cache")
        log_motion = None
        log_battery = None

        try:
            flight_phase = "CONNECTING"
            with SyncCrazyflie(DRONE_URI, cf=cf) as scf:
                scf_instance = scf
                flight_active = True

                # Setup logging
                flight_phase = "SETUP"
                log_motion, log_battery = setup_logging(cf)
                use_position_hold = log_motion is not None
                if use_position_hold:
                    time.sleep(1.0)

                # Initialize flight (skip if in debug mode)
                if not DEBUG_MODE:
                    cf.commander.send_setpoint(0, 0, 0, 0)
                    time.sleep(0.1)
                    cf.param.set_value("commander.enHighLevel", "1")
                    time.sleep(0.5)
                else:
                    print("DEBUG MODE: Skipping flight initialization")

                # Takeoff (position integration disabled)
                flight_phase = "TAKEOFF"
                global position_integration_enabled
                position_integration_enabled = (
                    False  # Disable position integration during takeoff
                )
                if DEBUG_MODE:
                    print("DEBUG MODE: Simulating takeoff phase")
                start_time = time.time()
                init_csv_logging()
                while time.time() - start_time < TAKEOFF_TIME and flight_active:
                    if not DEBUG_MODE:
                        cf.commander.send_hover_setpoint(
                            TRIM_VX, TRIM_VY, 0, TARGET_HEIGHT
                        )  # Use global TARGET_HEIGHT
                    time.sleep(0.01)

                # Reset position tracking and enable integration for hover
                integrated_position_x = 0.0
                integrated_position_y = 0.0
                last_integration_time = time.time()
                last_reset_time = time.time()
                position_integration_enabled = (
                    True  # Enable position integration for hover
                )

                # Reset PID controller state
                global position_integral_x, position_integral_y, velocity_integral_x, velocity_integral_y
                global last_position_error_x, last_position_error_y, last_velocity_error_x, last_velocity_error_y
                position_integral_x = 0.0
                position_integral_y = 0.0
                velocity_integral_x = 0.0
                velocity_integral_y = 0.0
                last_position_error_x = 0.0
                last_position_error_y = 0.0
                last_velocity_error_x = 0.0
                last_velocity_error_y = 0.0

                # Position hold hover
                flight_phase = "HOVER"
                if DEBUG_MODE:
                    print("DEBUG MODE: Simulating hover phase")
                start_time = time.time()
                while (
                    time.time() - start_time < HOVER_DURATION and flight_active
                ):  # Use global HOVER_DURATION
                    if use_position_hold and sensor_data_ready:
                        motion_vx, motion_vy = calculate_position_hold_corrections()
                        # Check for periodic reset
                        if periodic_position_reset():
                            flight_phase = "HOVER (RESET)"
                        else:
                            flight_phase = "HOVER"
                    else:
                        motion_vx, motion_vy = 0.0, 0.0
                    log_to_csv()
                    # Apply corrections (note: axes swapped)
                    total_vx = TRIM_VX + motion_vy
                    total_vy = TRIM_VY + motion_vx
                    if not DEBUG_MODE:
                        cf.commander.send_hover_setpoint(
                            total_vx, total_vy, 0, TARGET_HEIGHT
                        )  # Use global TARGET_HEIGHT
                    time.sleep(CONTROL_UPDATE_RATE)

                # Landing
                flight_phase = "LANDING"
                if DEBUG_MODE:
                    print("DEBUG MODE: Simulating landing phase")
                start_time = time.time()
                while (
                    time.time() - start_time < LANDING_TIME and flight_active
                ):  # Use global LANDING_TIME
                    if not DEBUG_MODE:
                        cf.commander.send_hover_setpoint(TRIM_VX, TRIM_VY, 0, 0)
                    time.sleep(0.01)

                # Stop motors
                if not DEBUG_MODE:
                    cf.commander.send_setpoint(0, 0, 0, 0)
                flight_phase = "COMPLETE"

        except Exception as e:
            flight_phase = f"ERROR: {str(e)}"
        finally:
            # Stop logging
            close_csv_logging()
            if log_motion:
                try:
                    log_motion.stop()
                except:
                    pass
            if log_battery:
                try:
                    log_battery.stop()
                except:
                    pass
            flight_active = False
            self.flight_running = False
            self.start_button.config(state=tk.NORMAL)
            if flight_phase != "COMPLETE":
                self.status_var.set(f"Status: {flight_phase}")
            else:
                self.status_var.set("Status: Flight Complete")


def main():
    # Initialize CRTP drivers once at program start so GUI callbacks
    # can open Crazyflie links later (required by some cflib versions).
    try:
        cflib.crtp.init_drivers()
        print("cflib.crtp drivers initialized")
    except Exception as e:
        print(f"Warning: cflib.crtp.init_drivers() failed: {e}")

    root = tk.Tk()
    app = DeadReckoningGUI(root)

    def on_closing():
        global flight_active, sensor_test_active
        flight_active = False
        sensor_test_active = False
        # Try to stop/clear NeoPixel using existing Crazyflie link if available
        try:
            cf = getattr(scf_instance, "cf", None)
            if cf is not None:
                try:
                    np_stop_blink(cf)
                    np_clear(cf)
                except Exception:
                    pass
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
