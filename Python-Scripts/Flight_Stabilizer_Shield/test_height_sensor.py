"""Standalone height sensor test for the LiteWing flight stabilizer shield.

Streams altitude data (state estimator + range sensor) to the console and a
simple Tk GUI with a live plot to verify correct behaviour of the height inputs.
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


class HeightSensorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("LiteWing Height Sensor Test")
        self.root.geometry("1000x700")

        self.status_var = tk.StringVar(value="Status: Idle")
        self.est_height_var = tk.StringVar(value="Estimator Height: 0.000 m")
        self.range_height_var = tk.StringVar(value="Range Sensor: 0.000 m")

        self._build_controls()
        self._build_plot()

        self.stop_event = threading.Event()
        self.connection_thread: threading.Thread | None = None

        self.data_lock = threading.Lock()
        self.timestamps = deque(maxlen=HISTORY_LENGTH)
        self.est_history = deque(maxlen=HISTORY_LENGTH)
        self.range_history = deque(maxlen=HISTORY_LENGTH)
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

        tk.Label(value_frame, textvariable=self.est_height_var, font=("Arial", 12)).pack(side=tk.LEFT, padx=10)
        tk.Label(value_frame, textvariable=self.range_height_var, font=("Arial", 12)).pack(side=tk.LEFT, padx=10)

    def _build_plot(self) -> None:
        self.figure = Figure(figsize=(10, 6), dpi=100)
        self.axis = self.figure.add_subplot(1, 1, 1)
        self.axis.set_title("Height vs Time")
        self.axis.set_xlabel("Time (s)")
        self.axis.set_ylabel("Height (m)")
        self.axis.grid(True, alpha=0.3)

        (self.est_line,) = self.axis.plot([], [], label="Estimator", color="tab:blue")
        (self.range_line,) = self.axis.plot([], [], label="Range", color="tab:orange")
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

                log_config = LogConfig(name="HeightSensor", period_in_ms=LOG_PERIOD_MS)
                variables = [
                    ("stateEstimate.z", "float"),
                    ("range.zrange", "uint16_t"),
                ]

                if not self._add_variables_if_available(cf, log_config, variables):
                    self._set_status("Status: Height variables unavailable")
                    return

                log_config.data_received_cb.add_callback(self._log_callback)
                cf.log.add_config(log_config)
                log_config.start()
                print("[Height] Logging started")

                while not self.stop_event.is_set():
                    time.sleep(0.1)

                log_config.stop()
                print("[Height] Logging stopped")
        except Exception as exc:  # noqa: BLE001
            print(f"[Height] Connection error: {exc}")
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
                print(f"[Height] Logging {full_name}")
                added += 1
            else:
                print(f"[Height] Missing {full_name}")
        return added > 0

    def _log_callback(self, timestamp: int, data: dict, _: LogConfig) -> None:
        estimator_height = data.get("stateEstimate.z", 0.0)
        range_raw_mm = data.get("range.zrange", 0)
        range_height = range_raw_mm / 1000.0 if range_raw_mm else 0.0

        with self.data_lock:
            now = time.time()
            self.timestamps.append(now)
            self.est_history.append(estimator_height)
            self.range_history.append(range_height if range_raw_mm else None)
            self.latest_values = (estimator_height, range_height, bool(range_raw_mm))

        if time.time() - self.last_console_print >= 1.0:
            self.last_console_print = time.time()
            print(
                f"[Height] Estimator={estimator_height:.3f} m, "
                f"Range={range_height:.3f} m ({'valid' if range_raw_mm else 'invalid'})"
            )

    def _refresh_gui(self) -> None:
        with self.data_lock:
            if getattr(self, "latest_values", None):
                estimator_height, range_height, range_valid = self.latest_values
                self.est_height_var.set(f"Estimator Height: {estimator_height:.3f} m")
                if range_valid:
                    self.range_height_var.set(f"Range Sensor: {range_height:.3f} m")
                else:
                    self.range_height_var.set("Range Sensor: no reading")

                times = list(self.timestamps)
                if times:
                    t0 = times[0]
                    rel_times = [t - t0 for t in times]

                    est_vals = list(self.est_history)
                    range_vals = [val if val is not None else float("nan") for val in self.range_history]

                    self.est_line.set_data(rel_times, est_vals)
                    self.range_line.set_data(rel_times, range_vals)

                    last_time = rel_times[-1] if rel_times[-1] > 1 else 1
                    self.axis.set_xlim(max(0, last_time - 20), last_time + 1)

                    combined = [v for v in est_vals + range_vals if not (v != v)]  # filter NaN
                    if combined:
                        vmin = min(combined)
                        vmax = max(combined)
                    else:
                        vmin, vmax = 0.0, 1.0
                    margin = max(0.1, (vmax - vmin) * 0.2)
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
    app = HeightSensorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
