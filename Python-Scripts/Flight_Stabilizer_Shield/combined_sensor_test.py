"""Combined IMU and Height sensor test for the LiteWing drone.

This script connects to the drone, streams orientation, acceleration, and height data,
and shows the readings on both the console and a Tk GUI with side-by-side live plots.
"""

import threading
import time
from collections import deque
import tkinter as tk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

DRONE_URI = "udp://192.168.43.42"
LOG_PERIOD_MS = 50 
HISTORY_LENGTH = 400


class CombinedSensorApp:
    """
    GUI application to visualize IMU and Height sensor data side-by-side.

    Connects to the LiteWing drone, subscribes to IMU and height log variables,
    and displays them in two side-by-side plots for real-time monitoring.
    """
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("LiteWing Combined Sensor Test")
        self.root.geometry("1400x800")

        # Status and Telemetry variables
        self.status_var = tk.StringVar(value="Status: Idle")
        self.roll_var = tk.StringVar(value="Roll: 0.00°")
        self.pitch_var = tk.StringVar(value="Pitch: 0.00°")
        self.yaw_var = tk.StringVar(value="Yaw: 0.00°")
        self.acc_var = tk.StringVar(value="Accel XYZ: 0.00, 0.00, 0.00 m/s²")
        self.est_height_var = tk.StringVar(value="Estimator Height: 0.000 m")
        self.range_height_var = tk.StringVar(value="Range Sensor: 0.000 m")

        self._build_controls()
        self._build_plot()

        self.stop_event = threading.Event()
        self.connection_thread: threading.Thread | None = None

        self.data_lock = threading.Lock()
        
        # IMU Histories
        self.imu_timestamps = deque(maxlen=HISTORY_LENGTH)
        self.roll_history = deque(maxlen=HISTORY_LENGTH)
        self.pitch_history = deque(maxlen=HISTORY_LENGTH)
        self.yaw_history = deque(maxlen=HISTORY_LENGTH)
        
        # Height Histories
        self.height_timestamps = deque(maxlen=HISTORY_LENGTH)
        self.est_history = deque(maxlen=HISTORY_LENGTH)
        self.range_history = deque(maxlen=HISTORY_LENGTH)
        
        # Current state values
        self.curr_roll = 0.0
        self.curr_pitch = 0.0
        self.curr_yaw = 0.0
        self.curr_ax = 0.0
        self.curr_ay = 0.0
        self.curr_az = 0.0
        self.curr_est_z = 0.0
        self.curr_range_z = 0.0
        self.curr_range_valid = False
        
        self.last_console_print = 0.0
        self.latest_values = None

        self.root.after(100, self._refresh_gui)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_controls(self) -> None:
        """Create the top control bar with Start/Stop buttons and status/data labels."""
        top_frame = tk.Frame(self.root)
        top_frame.pack(fill=tk.X, padx=10, pady=6)

        tk.Button(top_frame, text="Start", command=self.start, bg="#28a745", fg="white", width=12).pack(side=tk.LEFT, padx=5)
        tk.Button(top_frame, text="Stop", command=self.stop, bg="#dc3545", fg="white", width=12).pack(side=tk.LEFT, padx=5)

        tk.Label(top_frame, textvariable=self.status_var, font=("Arial", 11, "bold"), fg="blue").pack(side=tk.LEFT, padx=20)

        data_frame = tk.Frame(self.root)
        data_frame.pack(fill=tk.X, padx=10, pady=6)

        # IMU Labels
        imu_group = tk.LabelFrame(data_frame, text="IMU / Orientation")
        imu_group.pack(side=tk.LEFT, padx=10, fill=tk.Y)
        tk.Label(imu_group, textvariable=self.roll_var, font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        tk.Label(imu_group, textvariable=self.pitch_var, font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        tk.Label(imu_group, textvariable=self.yaw_var, font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        tk.Label(imu_group, textvariable=self.acc_var, font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        # Height Labels
        height_group = tk.LabelFrame(data_frame, text="Height / Range")
        height_group.pack(side=tk.LEFT, padx=10, fill=tk.Y)
        tk.Label(height_group, textvariable=self.est_height_var, font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        tk.Label(height_group, textvariable=self.range_height_var, font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

    def _build_plot(self) -> None:
        """Set up the matplotlib figure with two side-by-side subplots."""
        self.figure = Figure(figsize=(14, 6), dpi=100)
        
        # Left Plot: Orientation
        self.ax_imu = self.figure.add_subplot(1, 2, 1)
        self.ax_imu.set_title("Orientation (Roll/Pitch/Yaw)")
        self.ax_imu.set_xlabel("Time (s)")
        self.ax_imu.set_ylabel("Angle (°)")
        self.ax_imu.grid(True, alpha=0.3)

        (self.roll_line,) = self.ax_imu.plot([], [], label="Roll", color="tab:red")
        (self.pitch_line,) = self.ax_imu.plot([], [], label="Pitch", color="tab:green")
        (self.yaw_line,) = self.ax_imu.plot([], [], label="Yaw", color="tab:blue")
        self.ax_imu.legend(loc="upper right")

        # Right Plot: Height
        self.ax_height = self.figure.add_subplot(1, 2, 2)
        self.ax_height.set_title("Height vs Time")
        self.ax_height.set_xlabel("Time (s)")
        self.ax_height.set_ylabel("Height (m)")
        self.ax_height.grid(True, alpha=0.3)

        (self.est_line,) = self.ax_height.plot([], [], label="Estimator", color="tab:blue")
        (self.range_line,) = self.ax_height.plot([], [], label="Range", color="tab:orange")
        self.ax_height.legend(loc="upper right")

        self.figure.tight_layout()

        canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.canvas = canvas

    def start(self) -> None:
        if self.connection_thread and self.connection_thread.is_alive():
            return
        self.stop_event.clear()
        self.connection_thread = threading.Thread(target=self._connection_worker, daemon=True)
        self.connection_thread.start()

    def stop(self) -> None:
        self.stop_event.set()

    def _connection_worker(self) -> None:
        self._set_status("Status: Connecting...")
        try:
            cflib.crtp.init_drivers(enable_debug_driver=False)
            with SyncCrazyflie(DRONE_URI, cf=Crazyflie(rw_cache="./cache")) as scf:
                cf = scf.cf
                self._set_status("Status: Connected")
                
                # Log configurations are limited in size (typically ~30 bytes).
                # To avoid "log configuration too large" errors, we split the 
                # variables into two separate configurations.
                imu_log = LogConfig(name="IMUData", period_in_ms=LOG_PERIOD_MS)
                imu_vars = [
                    ("stateEstimate.roll", "float"),
                    ("stateEstimate.pitch", "float"),
                    ("stateEstimate.yaw", "float"),
                    ("stateEstimate.ax", "float"),
                    ("stateEstimate.ay", "float"),
                    ("stateEstimate.az", "float"),
                ]

                height_log = LogConfig(name="HeightData", period_in_ms=LOG_PERIOD_MS)
                height_vars = [
                    ("stateEstimate.z", "float"),
                    ("range.zrange", "uint16_t"),
                ]

                # Add variables and register callbacks for both configs
                if self._add_variables_if_available(cf, imu_log, imu_vars):
                    imu_log.data_received_cb.add_callback(self._log_callback)
                    cf.log.add_config(imu_log)
                    imu_log.start()
                    print("[Combined] IMU logging started")

                if self._add_variables_if_available(cf, height_log, height_vars):
                    height_log.data_received_cb.add_callback(self._log_callback)
                    cf.log.add_config(height_log)
                    height_log.start()
                    print("[Combined] Height logging started")

                while not self.stop_event.is_set():
                    time.sleep(0.1)

                imu_log.stop()
                height_log.stop()
                print("[Combined] Logging stopped")
        except Exception as exc:
            print(f"[Combined] Connection error: {exc}")
            self._set_status("Status: Error - check console")
        finally:
            self._set_status("Status: Idle")

    def _add_variables_if_available(self, cf: Crazyflie, log_config: LogConfig, candidates: list[tuple[str, str]]) -> bool:
        toc = cf.log.toc.toc
        added = 0
        for full_name, var_type in candidates:
            group, name = full_name.split(".", maxsplit=1)
            if group in toc and name in toc[group]:
                log_config.add_variable(full_name, var_type)
                added += 1
            else:
                print(f"[Combined] Missing {full_name}")
        return added > 0

    def _log_callback(self, timestamp: int, data: dict, _: LogConfig) -> None:
        """Process incoming log data and update the corresponding history."""
        with self.data_lock:
            now = time.time()
            
            # Check for orientation data
            if "stateEstimate.roll" in data:
                self.curr_roll = data["stateEstimate.roll"]
                self.curr_pitch = data["stateEstimate.pitch"]
                self.curr_yaw = data["stateEstimate.yaw"]
                self.curr_ax = data.get("stateEstimate.ax", 0.0)
                self.curr_ay = data.get("stateEstimate.ay", 0.0)
                self.curr_az = data.get("stateEstimate.az", 0.0)
                
                # Update IMU History
                self.imu_timestamps.append(now)
                self.roll_history.append(self.curr_roll)
                self.pitch_history.append(self.curr_pitch)
                self.yaw_history.append(self.curr_yaw)
            
            # Check for height data
            if "stateEstimate.z" in data:
                self.curr_est_z = data["stateEstimate.z"]
                range_raw = data.get("range.zrange", 0)
                self.curr_range_z = range_raw / 1000.0
                self.curr_range_valid = (range_raw > 0)
                
                # Update Height History
                self.height_timestamps.append(now)
                self.est_history.append(self.curr_est_z)
                self.range_history.append(self.curr_range_z if self.curr_range_valid else None)

            # Update telemetry dictionary for GUI
            self.latest_values = {
                "roll": self.curr_roll, "pitch": self.curr_pitch, "yaw": self.curr_yaw,
                "ax": self.curr_ax, "ay": self.curr_ay, "az": self.curr_az,
                "est_z": self.curr_est_z,
                "range_z": self.curr_range_z,
                "range_valid": self.curr_range_valid
            }

        if time.time() - self.last_console_print >= 1.0:
            self.last_console_print = time.time()
            print(
                f"[Combined] RPY=({self.curr_roll:.1f}, {self.curr_pitch:.1f}, {self.curr_yaw:.1f})°, "
                f"Z=({self.curr_est_z:.3f}m, ToF={self.curr_range_z:.3f}m)"
            )

    def _refresh_gui(self) -> None:
        """Periodically refresh the GUI (every 100ms)."""
        with self.data_lock:
            if self.latest_values:
                # 1. Update UI Labels
                v = self.latest_values
                self.roll_var.set(f"Roll: {v['roll']:.2f}°")
                self.pitch_var.set(f"Pitch: {v['pitch']:.2f}°")
                self.yaw_var.set(f"Yaw: {v['yaw']:.2f}°")
                self.acc_var.set(f"Accel XYZ: {v['ax']:.2f}, {v['ay']:.2f}, {v['az']:.2f} m/s²")
                self.est_height_var.set(f"Estimator Height: {v['est_z']:.3f} m")
                
                if v['range_valid']:
                    self.range_height_var.set(f"Range Sensor: {v['range_z']:.3f} m")
                else:
                    self.range_height_var.set("Range Sensor: no reading")

                # 2. Update Plots with independent time axes
                # IMU Plot
                if self.imu_timestamps:
                    imu_times = list(self.imu_timestamps)
                    t0_imu = imu_times[0]
                    rel_imu_times = [t - t0_imu for t in imu_times]
                    
                    self.roll_line.set_data(rel_imu_times, list(self.roll_history))
                    self.pitch_line.set_data(rel_imu_times, list(self.pitch_history))
                    self.yaw_line.set_data(rel_imu_times, list(self.yaw_history))
                    
                    last_imu = rel_imu_times[-1] if rel_imu_times[-1] > 1 else 1
                    self.ax_imu.set_xlim(max(0, last_imu - 20), last_imu + 1)
                    
                    imu_all = list(self.roll_history) + list(self.pitch_history) + list(self.yaw_history)
                    if imu_all:
                        imin, imax = min(imu_all), max(imax for imax in imu_all) # fix: simple min/max
                        imargin = max(5, (imax - imin) * 0.2)
                        self.ax_imu.set_ylim(imin - imargin, imax + imargin)

                # Height Plot
                if self.height_timestamps:
                    h_times = list(self.height_timestamps)
                    t0_h = h_times[0]
                    rel_h_times = [t - t0_h for t in h_times]
                    
                    e_vals = list(self.est_history)
                    rng_vals = [val if val is not None else float("nan") for val in self.range_history]
                    
                    self.est_line.set_data(rel_h_times, e_vals)
                    self.range_line.set_data(rel_h_times, rng_vals)

                    last_h = rel_h_times[-1] if rel_h_times[-1] > 1 else 1
                    self.ax_height.set_xlim(max(0, last_h - 20), last_h + 1)
                    
                    h_all = [v for v in e_vals + rng_vals if v == v] # filter nan
                    if h_all:
                        hmin, hmax = min(h_all), max(h_all)
                        hmargin = max(0.1, (hmax - hmin) * 0.2)
                        self.ax_height.set_ylim(hmin - hmargin, hmax + hmargin)

                self.canvas.draw_idle()

        self.root.after(100, self._refresh_gui)

    def _set_status(self, text: str) -> None:
        self.status_var.set(text)

    def _on_close(self) -> None:
        self.stop()
        self.root.after(200, self.root.destroy)


def main() -> None:
    root = tk.Tk()
    app = CombinedSensorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
