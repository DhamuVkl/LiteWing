"""Combined Barometer and IMU sensor test for the LiteWing flight stabilization Module.

Streams data from both MS5611 (Barometer) and MPU6050 (IMU) simultaneously
and plots them side-by-side to allow correlation analysis.
"""

import sys
import tkinter as tk
from tkinter import messagebox

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
except ImportError as e:
    root = tk.Tk()
    root.withdraw()
    messagebox.showerror("Dependency Error", f"Matplotlib is missing.\n\nError: {e}\n\nPlease install it: pip install matplotlib")
    sys.exit(1)

try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
except ImportError as e:
    root = tk.Tk()
    root.withdraw()
    messagebox.showerror("Dependency Error", f"cflib is missing.\n\nError: {e}\n\nPlease install it: pip install cflib")
    sys.exit(1)

import threading
import time
from collections import deque

# Configuration
DRONE_URI = "udp://192.168.43.42"
LOG_PERIOD_MS = 50  # 20 Hz
HISTORY_LENGTH = 400


class CombinedSensorApp:
    """
    GUI application to visualize both Barometer and IMU data.
    """
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("LiteWing Sensor Fusion Test (Baro + IMU)")
        self.root.geometry("1400x900")

        # Variables for GUI Labels
        self.status_var = tk.StringVar(value="Status: Idle")
        
        # Baro Variables
        self.baro_pressure_var = tk.StringVar(value="Pres: 0.00 mbar")
        self.baro_asl_var = tk.StringVar(value="ASL: 0.00 m")
        self.baro_temp_var = tk.StringVar(value="Temp: 0.00 C")
        
        # IMU Variables
        self.imu_roll_var = tk.StringVar(value="R: 0.00°")
        self.imu_pitch_var = tk.StringVar(value="P: 0.00°")
        self.imu_yaw_var = tk.StringVar(value="Y: 0.00°")
        self.imu_acc_var = tk.StringVar(value="Acc: 0.00, 0.00, 0.00")

        self._build_controls()
        self._build_plots()

        self.stop_event = threading.Event()
        self.connection_thread: threading.Thread | None = None

        self.data_lock = threading.Lock()
        
        # Baro History
        self.baro_timestamps = deque(maxlen=HISTORY_LENGTH)
        self.pressure_history = deque(maxlen=HISTORY_LENGTH)
        self.asl_history = deque(maxlen=HISTORY_LENGTH)
        
        # IMU History
        self.imu_timestamps = deque(maxlen=HISTORY_LENGTH)
        self.roll_history = deque(maxlen=HISTORY_LENGTH)
        self.pitch_history = deque(maxlen=HISTORY_LENGTH)
        self.yaw_history = deque(maxlen=HISTORY_LENGTH)
        self.ax_history = deque(maxlen=HISTORY_LENGTH)
        self.ay_history = deque(maxlen=HISTORY_LENGTH)
        self.az_history = deque(maxlen=HISTORY_LENGTH)

        self.last_console_print = 0.0

        self.root.after(100, self._refresh_gui)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_controls(self) -> None:
        """Top control bar."""
        top_frame = tk.Frame(self.root)
        top_frame.pack(fill=tk.X, padx=10, pady=6)

        tk.Button(top_frame, text="Start", command=self.start, bg="#28a745", fg="white", width=12).pack(side=tk.LEFT, padx=5)
        tk.Button(top_frame, text="Stop", command=self.stop, bg="#dc3545", fg="white", width=12).pack(side=tk.LEFT, padx=5)
        tk.Label(top_frame, textvariable=self.status_var, font=("Arial", 11, "bold"), fg="blue").pack(side=tk.LEFT, padx=20)

        # Data Display Frame
        data_frame = tk.Frame(self.root)
        data_frame.pack(fill=tk.X, padx=10, pady=2)
        
        # Group 1: Barometer
        lbl_baro = tk.LabelFrame(data_frame, text="Barometer (MS5611)", font=("Arial", 10, "bold"), padx=5, pady=5)
        lbl_baro.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        tk.Label(lbl_baro, textvariable=self.baro_pressure_var, font=("Consolas", 11)).pack(side=tk.LEFT, padx=5)
        tk.Label(lbl_baro, textvariable=self.baro_asl_var, font=("Consolas", 11)).pack(side=tk.LEFT, padx=5)
        tk.Label(lbl_baro, textvariable=self.baro_temp_var, font=("Consolas", 11)).pack(side=tk.LEFT, padx=5)

        # Group 2: IMU
        lbl_imu = tk.LabelFrame(data_frame, text="IMU (MPU6050)", font=("Arial", 10, "bold"), padx=5, pady=5)
        lbl_imu.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        tk.Label(lbl_imu, textvariable=self.imu_roll_var, font=("Consolas", 11)).pack(side=tk.LEFT, padx=5)
        tk.Label(lbl_imu, textvariable=self.imu_pitch_var, font=("Consolas", 11)).pack(side=tk.LEFT, padx=5)
        tk.Label(lbl_imu, textvariable=self.imu_yaw_var, font=("Consolas", 11)).pack(side=tk.LEFT, padx=5)
        tk.Label(lbl_imu, textvariable=self.imu_acc_var, font=("Consolas", 11)).pack(side=tk.LEFT, padx=5)

    def _build_plots(self) -> None:
        """2x2 Grid of plots."""
        self.figure = Figure(figsize=(14, 8), dpi=100)
        
        # --- LEFT COLUMN: BAROMETER ---
        
        # Plot 1: Pressure
        self.ax_pres = self.figure.add_subplot(2, 2, 1)
        self.ax_pres.set_title("Barometric Pressure")
        self.ax_pres.set_ylabel("mbar")
        self.ax_pres.grid(True, alpha=0.3)
        (self.line_pres,) = self.ax_pres.plot([], [], label="Pressure", color="tab:blue")
        
        # Plot 2: ASL
        self.ax_asl = self.figure.add_subplot(2, 2, 3, sharex=self.ax_pres)
        self.ax_asl.set_title("Altitude (ASL)")
        self.ax_asl.set_xlabel("Time (s)")
        self.ax_asl.set_ylabel("Meters")
        self.ax_asl.grid(True, alpha=0.3)
        (self.line_asl,) = self.ax_asl.plot([], [], label="ASL", color="tab:orange")

        # --- RIGHT COLUMN: IMU ---
        
        # Plot 3: Orientation (Roll/Pitch)
        # Note: Yaw often drifts or wraps, but we'll plot it.
        self.ax_att = self.figure.add_subplot(2, 2, 2) # Independent X axis
        self.ax_att.set_title("Attitude (Roll / Pitch)")
        self.ax_att.set_ylabel("Degrees")
        self.ax_att.grid(True, alpha=0.3)
        (self.line_roll,) = self.ax_att.plot([], [], label="Roll", color="tab:red")
        (self.line_pitch,) = self.ax_att.plot([], [], label="Pitch", color="tab:green")
        self.ax_att.legend(loc="upper right", fontsize="small")

        # Plot 4: Acceleration
        self.ax_acc = self.figure.add_subplot(2, 2, 4, sharex=self.ax_att)
        self.ax_acc.set_title("Acceleration (Z-axis)") # Focus on Z for hovering usually, or plot all.
        # Let's plot Z mainly, maybe X/Y in background? Let's plot all for completeness manually.
        self.ax_acc.set_xlabel("Time (s)")
        self.ax_acc.set_ylabel("g (approx)")
        self.ax_acc.grid(True, alpha=0.3)
        (self.line_ax,) = self.ax_acc.plot([], [], label="AX", color="salmon", alpha=0.6, linewidth=1)
        (self.line_ay,) = self.ax_acc.plot([], [], label="AY", color="lightgreen", alpha=0.6, linewidth=1)
        (self.line_az,) = self.ax_acc.plot([], [], label="AZ", color="tab:purple", linewidth=1.5)
        self.ax_acc.legend(loc="upper right", fontsize="small")
        
        self.figure.tight_layout()

        canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
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

                # --- SPLIT CONFIGS TO AVOID PACKET SIZE LIMITS ---
                
                # Config 1: Barometer
                log_baro = LogConfig(name="Baro", period_in_ms=LOG_PERIOD_MS)
                log_baro.add_variable("baro.pressure", "float")
                log_baro.add_variable("baro.asl", "float")
                log_baro.add_variable("baro.temp", "float")
                
                # Config 2: IMU
                log_imu = LogConfig(name="IMU", period_in_ms=LOG_PERIOD_MS)
                log_imu.add_variable("stateEstimate.roll", "float")
                log_imu.add_variable("stateEstimate.pitch", "float")
                log_imu.add_variable("stateEstimate.yaw", "float")
                log_imu.add_variable("stateEstimate.ax", "float")
                log_imu.add_variable("stateEstimate.ay", "float")
                log_imu.add_variable("stateEstimate.az", "float")

                # Add and start
                # We assume these variables exist somewhat blindly here or use try/except add
                # For robustness, we can check TOC but let's trust the names are standard now.
                
                log_baro.data_received_cb.add_callback(self._baro_cb)
                log_imu.data_received_cb.add_callback(self._imu_cb)
                
                cf.log.add_config(log_baro)
                cf.log.add_config(log_imu)
                
                log_baro.start()
                log_imu.start()
                
                print("[Sys] Logging started (Split Configs)")

                while not self.stop_event.is_set():
                    time.sleep(0.1)

                log_baro.stop()
                log_imu.stop()
                print("[Sys] Logging stopped")

        except Exception as exc:
            print(f"[Sys] Connection error: {exc}")
            self._set_status(f"Status: Error - {exc}")
        finally:
            self._set_status("Status: Idle")

    def _baro_cb(self, timestamp: int, data: dict, _: LogConfig) -> None:
        b_pres = data.get("baro.pressure", 0.0)
        b_asl = data.get("baro.asl", 0.0)
        b_temp = data.get("baro.temp", 0.0)
        
        with self.data_lock:
            now = time.time()
            self.baro_timestamps.append(now)
            self.pressure_history.append(b_pres)
            self.asl_history.append(b_asl)
            self.latest_baro = (b_pres, b_asl, b_temp)

    def _imu_cb(self, timestamp: int, data: dict, _: LogConfig) -> None:
        i_roll = data.get("stateEstimate.roll", 0.0)
        i_pitch = data.get("stateEstimate.pitch", 0.0)
        i_yaw = data.get("stateEstimate.yaw", 0.0)
        i_ax = data.get("stateEstimate.ax", 0.0)
        i_ay = data.get("stateEstimate.ay", 0.0)
        i_az = data.get("stateEstimate.az", 0.0)
        
        with self.data_lock:
            now = time.time()
            self.imu_timestamps.append(now)
            self.roll_history.append(i_roll)
            self.pitch_history.append(i_pitch)
            self.yaw_history.append(i_yaw)
            self.ax_history.append(i_ax)
            self.ay_history.append(i_ay)
            self.az_history.append(i_az)
            self.latest_imu = (i_roll, i_pitch, i_yaw, i_ax, i_ay, i_az)

    def _refresh_gui(self) -> None:
        with self.data_lock:
            # Update Text Labels
            if hasattr(self, "latest_baro"):
                p, a, t = self.latest_baro
                self.baro_pressure_var.set(f"Pres: {p:.2f} mbar")
                self.baro_asl_var.set(f"ASL: {a:.2f} m")
                self.baro_temp_var.set(f"Temp: {t:.2f} C")
            
            if hasattr(self, "latest_imu"):
                r, p, y, ax, ay, az = self.latest_imu
                self.imu_roll_var.set(f"R: {r:.2f}°")
                self.imu_pitch_var.set(f"P: {p:.2f}°")
                self.imu_yaw_var.set(f"Y: {y:.2f}°")
                self.imu_acc_var.set(f"Acc: {ax:.1f}, {ay:.1f}, {az:.1f}")

            # --- Update Baro Plots ---
            times_b = list(self.baro_timestamps)
            if times_b:
                t0 = times_b[0]
                rel_times = [t - t0 for t in times_b]
                self.line_pres.set_data(rel_times, list(self.pressure_history))
                self.line_asl.set_data(rel_times, list(self.asl_history))
                
                # Auto-scale Baro
                last_time = rel_times[-1] if rel_times[-1] > 1 else 1
                self.ax_pres.set_xlim(max(0, last_time - 20), last_time + 1)
                
                if self.pressure_history:
                    self._auto_scale(self.ax_pres, self.pressure_history)
                if self.asl_history:
                    self._auto_scale(self.ax_asl, self.asl_history)

            # --- Update IMU Plots ---
            times_i = list(self.imu_timestamps)
            if times_i:
                # Use Baro start time as reference if available, else IMU start
                t0_ref = times_b[0] if times_b else times_i[0]
                rel_times = [t - t0_ref for t in times_i]
                
                self.line_roll.set_data(rel_times, list(self.roll_history))
                self.line_pitch.set_data(rel_times, list(self.pitch_history))
                
                self.line_ax.set_data(rel_times, list(self.ax_history))
                self.line_ay.set_data(rel_times, list(self.ay_history))
                self.line_az.set_data(rel_times, list(self.az_history))
                
                # Auto-scale IMU
                last_time = rel_times[-1] if rel_times[-1] > 1 else 1
                self.ax_att.set_xlim(max(0, last_time - 20), last_time + 1)

                if self.roll_history:
                    self._auto_scale(self.ax_att, list(self.roll_history) + list(self.pitch_history), default_margin=5)
                if self.ax_history:
                    self._auto_scale(self.ax_acc, list(self.ax_history) + list(self.ay_history) + list(self.az_history), default_margin=0.2)

            self.canvas.draw_idle()

        self.root.after(100, self._refresh_gui)

    def _auto_scale(self, ax, data, default_margin=None):
        vmin, vmax = min(data), max(data)
        rng = vmax - vmin
        if rng == 0:
            margin = 1.0 if default_margin is None else default_margin
        else:
            margin = rng * 0.2
            if default_margin and margin < default_margin:
                margin = default_margin
        ax.set_ylim(vmin - margin, vmax + margin)

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
