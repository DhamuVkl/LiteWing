"""Standalone barometric pressure sensor test for the LiteWing flight stabilization Module.

Streams baro data (pressure, temperature, ASL) to the console and a
simple Tk GUI with a live plot.
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


class BaroSensorApp:
    """
    Simple GUI to display the MS5611 data.
    """
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("LiteWing Baro Sensor Test")
        self.root.geometry("1000x700")

        self.status_var = tk.StringVar(value="Status: Idle")
        self.pressure_var = tk.StringVar(value="Pressure: 0.00 mbar")
        self.temp_var = tk.StringVar(value="Temp: 0.00 C")
        self.asl_var = tk.StringVar(value="ASL: 0.00 m")

        self._build_controls()
        self._build_plot()

        self.stop_event = threading.Event()
        self.connection_thread: threading.Thread | None = None

        self.data_lock = threading.Lock()
        self.timestamps = deque(maxlen=HISTORY_LENGTH)
        self.pressure_history = deque(maxlen=HISTORY_LENGTH)
        self.asl_history = deque(maxlen=HISTORY_LENGTH)
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

        tk.Label(value_frame, textvariable=self.pressure_var, font=("Arial", 12)).pack(side=tk.LEFT, padx=10)
        tk.Label(value_frame, textvariable=self.temp_var, font=("Arial", 12)).pack(side=tk.LEFT, padx=10)
        tk.Label(value_frame, textvariable=self.asl_var, font=("Arial", 12)).pack(side=tk.LEFT, padx=10)

    def _build_plot(self) -> None:
        self.figure = Figure(figsize=(10, 6), dpi=100)
        self.axis = self.figure.add_subplot(1, 1, 1)
        self.axis.set_title("Barometer Data vs Time")
        self.axis.set_xlabel("Time (s)")
        self.axis.set_ylabel("Pressure (mbar) / ASL (m)")
        self.axis.grid(True, alpha=0.3)

        # Plot Pressure on left axis, create secondary axis for ASL?
        # Or just plot ASL since user cares about height using MS5611.
        # User asked to "measure the height".
        
        self.asl_line, = self.axis.plot([], [], label="ASL (m)", color="tab:blue")
        # Scale pressure to fit or use twinx? For simplicity, let's plot ASL primarily.
        
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

                # Try 'baro.pressure', 'baro.temp', 'baro.asl'
                log_config = LogConfig(name="BaroSensor", period_in_ms=LOG_PERIOD_MS)
                variables = [
                    ("baro.pressure", "float"),
                    ("baro.temp", "float"),
                    ("baro.asl", "float"),
                ]

                if not self._add_variables_if_available(cf, log_config, variables):
                    self._set_status("Status: Baro variables unavailable")
                    return

                log_config.data_received_cb.add_callback(self._log_callback)
                cf.log.add_config(log_config)
                log_config.start()
                print("[Baro] Logging started")

                while not self.stop_event.is_set():
                    time.sleep(0.1)

                log_config.stop()
                print("[Baro] Logging stopped")
        except Exception as exc:
            print(f"[Baro] Connection error: {exc}")
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
                print(f"[Baro] Logging {full_name}")
                added += 1
            else:
                print(f"[Baro] Missing {full_name}")
        return added > 0

    def _log_callback(self, timestamp: int, data: dict, _: LogConfig) -> None:
        pressure = data.get("baro.pressure", 0.0)
        temp = data.get("baro.temp", 0.0)
        asl = data.get("baro.asl", 0.0)

        with self.data_lock:
            now = time.time()
            self.timestamps.append(now)
            self.pressure_history.append(pressure)
            self.asl_history.append(asl)
            self.latest_values = (pressure, temp, asl)

        if time.time() - self.last_console_print >= 1.0:
            self.last_console_print = time.time()
            print(f"[Baro] Pressure={pressure:.2f} mbar, Temp={temp:.2f} C, ASL={asl:.2f} m")

    def _refresh_gui(self) -> None:
        with self.data_lock:
            if getattr(self, "latest_values", None):
                p, t, a = self.latest_values
                self.pressure_var.set(f"Pressure: {p:.2f} mbar")
                self.temp_var.set(f"Temp: {t:.2f} C")
                self.asl_var.set(f"ASL: {a:.2f} m")

                times = list(self.timestamps)
                if times:
                    t0 = times[0]
                    rel_times = [x - t0 for x in times]
                    asl_vals = list(self.asl_history)
                    
                    self.asl_line.set_data(rel_times, asl_vals)

                    last_time = rel_times[-1] if rel_times[-1] > 1 else 1
                    self.axis.set_xlim(max(0, last_time - 20), last_time + 1)
                    
                    if asl_vals:
                        valid_vals = [v for v in asl_vals if v == v] # filter nan if any
                        if valid_vals:
                            vmin, vmax = min(valid_vals), max(valid_vals)
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
    app = BaroSensorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
