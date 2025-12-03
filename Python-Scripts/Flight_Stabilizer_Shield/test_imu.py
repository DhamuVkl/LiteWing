"""Standalone IMU sensor test for the LiteWing flight stabilizer shield.

This script connects to the drone, streams orientation and acceleration data, and
shows the readings on both the console and a simple Tk GUI with live plots.
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
LOG_PERIOD_MS = 50  # 20 Hz is sufficient for visualization
HISTORY_LENGTH = 400


class IMUTestApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("LiteWing IMU Test")
        self.root.geometry("1100x720")

        self.status_var = tk.StringVar(value="Status: Idle")
        self.roll_var = tk.StringVar(value="Roll: 0.00°")
        self.pitch_var = tk.StringVar(value="Pitch: 0.00°")
        self.yaw_var = tk.StringVar(value="Yaw: 0.00°")
        self.acc_var = tk.StringVar(value="Accel XYZ: 0.00, 0.00, 0.00 m/s²")

        self._build_controls()
        self._build_plot()

        self.stop_event = threading.Event()
        self.connection_thread: threading.Thread | None = None

        self.data_lock = threading.Lock()
        self.timestamps = deque(maxlen=HISTORY_LENGTH)
        self.roll_history = deque(maxlen=HISTORY_LENGTH)
        self.pitch_history = deque(maxlen=HISTORY_LENGTH)
        self.yaw_history = deque(maxlen=HISTORY_LENGTH)
        self.last_console_print = 0.0

        self.root.after(100, self._refresh_gui)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_controls(self) -> None:
        top_frame = tk.Frame(self.root)
        top_frame.pack(fill=tk.X, padx=10, pady=6)

        tk.Button(top_frame, text="Start", command=self.start, bg="#28a745", fg="white", width=12).pack(side=tk.LEFT, padx=5)
        tk.Button(top_frame, text="Stop", command=self.stop, bg="#dc3545", fg="white", width=12).pack(side=tk.LEFT, padx=5)

        tk.Label(top_frame, textvariable=self.status_var, font=("Arial", 11, "bold"), fg="blue").pack(side=tk.LEFT, padx=20)

        value_frame = tk.Frame(self.root)
        value_frame.pack(fill=tk.X, padx=10, pady=6)

        tk.Label(value_frame, textvariable=self.roll_var, font=("Arial", 12)).pack(side=tk.LEFT, padx=10)
        tk.Label(value_frame, textvariable=self.pitch_var, font=("Arial", 12)).pack(side=tk.LEFT, padx=10)
        tk.Label(value_frame, textvariable=self.yaw_var, font=("Arial", 12)).pack(side=tk.LEFT, padx=10)
        tk.Label(value_frame, textvariable=self.acc_var, font=("Arial", 12)).pack(side=tk.LEFT, padx=10)

    def _build_plot(self) -> None:
        self.figure = Figure(figsize=(11, 6), dpi=100)
        self.axis = self.figure.add_subplot(1, 1, 1)
        self.axis.set_title("Orientation (Roll/Pitch/Yaw)")
        self.axis.set_xlabel("Time (s)")
        self.axis.set_ylabel("Angle (°)")
        self.axis.grid(True, alpha=0.3)

        (self.roll_line,) = self.axis.plot([], [], label="Roll", color="tab:red")
        (self.pitch_line,) = self.axis.plot([], [], label="Pitch", color="tab:green")
        (self.yaw_line,) = self.axis.plot([], [], label="Yaw", color="tab:blue")
        self.axis.legend(loc="upper right")

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
                log_config = LogConfig(name="IMUSensor", period_in_ms=LOG_PERIOD_MS)

                available_vars = [
                    ("stateEstimate.roll", "float"),
                    ("stateEstimate.pitch", "float"),
                    ("stateEstimate.yaw", "float"),
                    ("stateEstimate.ax", "float"),
                    ("stateEstimate.ay", "float"),
                    ("stateEstimate.az", "float"),
                ]

                if not self._add_variables_if_available(cf, log_config, available_vars):
                    self._set_status("Status: IMU variables unavailable")
                    return

                log_config.data_received_cb.add_callback(self._log_callback)
                cf.log.add_config(log_config)
                log_config.start()
                print("[IMU] Logging started")

                while not self.stop_event.is_set():
                    time.sleep(0.1)

                log_config.stop()
                print("[IMU] Logging stopped")
        except Exception as exc:  # noqa: BLE001
            print(f"[IMU] Connection error: {exc}")
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
                print(f"[IMU] Logging {full_name}")
                added += 1
            else:
                print(f"[IMU] Missing {full_name}")
        return added > 0

    def _log_callback(self, timestamp: int, data: dict, _: LogConfig) -> None:
        roll = data.get("stateEstimate.roll", 0.0)
        pitch = data.get("stateEstimate.pitch", 0.0)
        yaw = data.get("stateEstimate.yaw", 0.0)
        ax = data.get("stateEstimate.ax", 0.0)
        ay = data.get("stateEstimate.ay", 0.0)
        az = data.get("stateEstimate.az", 0.0)

        with self.data_lock:
            now = time.time()
            self.timestamps.append(now)
            self.roll_history.append(roll)
            self.pitch_history.append(pitch)
            self.yaw_history.append(yaw)
            self.latest_values = (roll, pitch, yaw, ax, ay, az)

        if time.time() - self.last_console_print >= 1.0:
            self.last_console_print = time.time()
            print(
                f"[IMU] Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°, "
                f"Accel=({ax:.2f}, {ay:.2f}, {az:.2f}) m/s²"
            )

    def _refresh_gui(self) -> None:
        with self.data_lock:
            if getattr(self, "latest_values", None):
                roll, pitch, yaw, ax, ay, az = self.latest_values
                self.roll_var.set(f"Roll: {roll:.2f}°")
                self.pitch_var.set(f"Pitch: {pitch:.2f}°")
                self.yaw_var.set(f"Yaw: {yaw:.2f}°")
                self.acc_var.set(f"Accel XYZ: {ax:.2f}, {ay:.2f}, {az:.2f} m/s²")

                times = list(self.timestamps)
                if times:
                    t0 = times[0]
                    rel_times = [t - t0 for t in times]
                    roll_vals = list(self.roll_history)
                    pitch_vals = list(self.pitch_history)
                    yaw_vals = list(self.yaw_history)

                    self.roll_line.set_data(rel_times, roll_vals)
                    self.pitch_line.set_data(rel_times, pitch_vals)
                    self.yaw_line.set_data(rel_times, yaw_vals)

                    last_time = rel_times[-1] if rel_times[-1] > 1 else 1
                    self.axis.set_xlim(max(0, last_time - 20), last_time + 1)

                    all_vals = roll_vals + pitch_vals + yaw_vals
                    vmin = min(all_vals) if all_vals else -5
                    vmax = max(all_vals) if all_vals else 5
                    margin = max(5, (vmax - vmin) * 0.2)
                    self.axis.set_ylim(vmin - margin, vmax + margin)

                    self.canvas.draw_idle()

        self.root.after(100, self._refresh_gui)

    def _set_status(self, text: str) -> None:
        self.status_var.set(text)

    def _on_close(self) -> None:
        self.stop()
        self.root.after(200, self.root.destroy)


def main() -> None:
    root = tk.Tk()
    app = IMUTestApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
