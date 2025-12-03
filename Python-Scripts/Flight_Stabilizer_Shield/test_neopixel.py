"""Standalone NeoPixel LED test for the LiteWing flight stabilizer shield.

Provides a small GUI to connect to the drone, set colours, clear LEDs, and toggle
blinking. All commands are echoed to both the console and the GUI log window.
"""

import threading
import time
import tkinter as tk
from tkinter import ttk

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

DRONE_URI = "udp://192.168.43.42"
CRTP_PORT_NEOPIXEL = 0x09
NEOPIXEL_CHANNEL_SET_PIXEL = 0x00
NEOPIXEL_CHANNEL_SHOW = 0x01
NEOPIXEL_CHANNEL_CLEAR = 0x02
NEOPIXEL_CHANNEL_BLINK = 0x03
NP_SEND_RETRIES = 3
NP_PACKET_DELAY = 0.02
NP_LINK_SETUP_DELAY = 0.1


def _send_crtp_with_fallback(cf: Crazyflie, port: int, channel: int, payload: bytes) -> None:
    header = ((port & 0x0F) << 4) | (channel & 0x0F)

    class _Packet:
        def __init__(self, header_value: int, data: bytes):
            self._header = header_value
            self._data = data

        def is_data_size_valid(self) -> bool:
            return len(self._data) <= 30

        @property
        def data(self) -> bytes:
            return self._data

        @property
        def header(self) -> int:
            return self._header

        @property
        def raw(self) -> bytes:
            return bytes([self._header]) + self._data

    packet = _Packet(header, payload)

    try:
        send_packet = getattr(cf, "send_packet", None)
        if callable(send_packet):
            send_packet(packet)
            return
    except Exception:  # noqa: BLE001
        pass

    try:
        link = getattr(cf, "_link", None) or getattr(cf, "link", None)
        if link is not None and callable(getattr(link, "send_packet", None)):
            link.send_packet(packet)
            return
    except Exception:  # noqa: BLE001
        pass

    try:
        from cflib import crtp as _crtp  # Local import to avoid global dependency

        sendp = getattr(_crtp, "send_packet", None)
        if callable(sendp):
            # cflib expects either packets with .raw or bytes
            try:
                sendp(packet)
            except TypeError:
                sendp(packet.raw)
            return
    except Exception:  # noqa: BLE001
        pass

    raise RuntimeError("Unable to send CRTP NeoPixel packet")


def np_set_pixel(cf: Crazyflie, index: int, r: int, g: int, b: int) -> None:
    payload = bytes([index & 0xFF, r & 0xFF, g & 0xFF, b & 0xFF])
    _send_crtp_with_fallback(cf, CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_SET_PIXEL, payload)


def np_set_all(cf: Crazyflie, r: int, g: int, b: int) -> None:
    payload = bytes([0xFF, r & 0xFF, g & 0xFF, b & 0xFF])
    _send_crtp_with_fallback(cf, CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_SET_PIXEL, payload)


def np_show(cf: Crazyflie) -> None:
    _send_crtp_with_fallback(cf, CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_SHOW, b"")


def np_clear(cf: Crazyflie) -> None:
    _send_crtp_with_fallback(cf, CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_CLEAR, b"")


def np_start_blink(cf: Crazyflie, on_ms: int = 500, off_ms: int = 500) -> None:
    payload = bytes([
        1,
        (on_ms >> 8) & 0xFF,
        on_ms & 0xFF,
        (off_ms >> 8) & 0xFF,
        off_ms & 0xFF,
    ])
    _send_crtp_with_fallback(cf, CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_BLINK, payload)


def np_stop_blink(cf: Crazyflie) -> None:
    _send_crtp_with_fallback(cf, CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_BLINK, bytes([0, 0, 0, 0, 0]))


def _try_send_with_retries(cf: Crazyflie, func, *args, retries: int = NP_SEND_RETRIES, logger=None) -> bool:
    last_exc: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            func(cf, *args)
            time.sleep(NP_PACKET_DELAY)
            return True
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
            if logger:
                logger(f"Attempt {attempt} failed: {exc}")
            time.sleep(NP_PACKET_DELAY)
    if logger:
        logger(f"Command failed after {retries} retries: {last_exc}")
    return False


class NeoPixelApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("LiteWing NeoPixel Test")
        self.root.geometry("760x520")

        self.status_var = tk.StringVar(value="Status: Disconnected")
        self.blinking = False
        self.scf: SyncCrazyflie | None = None
        self.cf: Crazyflie | None = None

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        control_frame = tk.Frame(self.root)
        control_frame.pack(fill=tk.X, padx=10, pady=6)

        ttk.Button(control_frame, text="Connect", command=self.connect, width=12).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Disconnect", command=self.disconnect, width=12).pack(side=tk.LEFT, padx=5)
        ttk.Label(control_frame, textvariable=self.status_var, font=("Arial", 11, "bold"), foreground="blue").pack(side=tk.LEFT, padx=20)

        spin_frame = ttk.LabelFrame(self.root, text="Colour Controls")
        spin_frame.pack(fill=tk.X, padx=10, pady=6)

        self.r_var = tk.IntVar(value=255)
        self.g_var = tk.IntVar(value=255)
        self.b_var = tk.IntVar(value=255)
        self.pixel_index_var = tk.IntVar(value=-1)

        for label_text, var in (("R", self.r_var), ("G", self.g_var), ("B", self.b_var)):
            frame = tk.Frame(spin_frame)
            frame.pack(side=tk.LEFT, padx=6)
            ttk.Label(frame, text=f"{label_text}:").pack(side=tk.LEFT)
            ttk.Spinbox(frame, from_=0, to=255, textvariable=var, width=5).pack(side=tk.LEFT)

        index_frame = tk.Frame(spin_frame)
        index_frame.pack(side=tk.LEFT, padx=6)
        ttk.Label(index_frame, text="Pixel index (-1=all):").pack(side=tk.LEFT)
        ttk.Spinbox(index_frame, from_=-1, to=255, textvariable=self.pixel_index_var, width=6).pack(side=tk.LEFT)

        button_frame = ttk.Frame(self.root)
        button_frame.pack(fill=tk.X, padx=10, pady=6)

        ttk.Button(button_frame, text="Set Colour", command=self.set_colour, width=18).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Show", command=self.show_colour, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Clear", command=self.clear_leds, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Toggle Blink", command=self.toggle_blink, width=14).pack(side=tk.LEFT, padx=5)

        log_frame = ttk.LabelFrame(self.root, text="Command Log")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=6)

        self.log_list = tk.Listbox(log_frame, font=("Consolas", 10))
        self.log_list.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

    def connect(self) -> None:
        if self.cf is not None:
            self._log("Already connected")
            return

        def worker() -> None:
            self._set_status("Status: Connecting...")
            try:
                cflib.crtp.init_drivers(enable_debug_driver=False)
                scf = SyncCrazyflie(DRONE_URI, cf=Crazyflie(rw_cache="./cache"))
                scf.open_link()
                time.sleep(NP_LINK_SETUP_DELAY)
                self.scf = scf
                self.cf = scf.cf
                self._set_status("Status: Connected")
                self._log("Connected to drone")
            except Exception as exc:  # noqa: BLE001
                self._set_status("Status: Error - see console")
                self._log(f"Connection failed: {exc}")
                print(f"[NeoPixel] Connection failed: {exc}")
                self.scf = None
                self.cf = None

        threading.Thread(target=worker, daemon=True).start()

    def disconnect(self) -> None:
        if self.scf is None:
            self._log("Not connected")
            return

        def worker() -> None:
            try:
                if self.cf is not None and self.blinking:
                    _try_send_with_retries(self.cf, np_stop_blink, logger=self._log)
                if self.scf is not None:
                    self.scf.close_link()
            except Exception as exc:  # noqa: BLE001
                self._log(f"Disconnect error: {exc}")
            finally:
                self.scf = None
                self.cf = None
                self.blinking = False
                self._set_status("Status: Disconnected")
                self._log("Disconnected")

        threading.Thread(target=worker, daemon=True).start()

    def set_colour(self) -> None:
        cf = self.cf
        if cf is None:
            self._log("Set colour requested without connection")
            return
        r, g, b = self._clamp_rgb()
        pixel_index = self.pixel_index_var.get()
        if pixel_index < 0:
            ok = _try_send_with_retries(cf, np_set_all, r, g, b, logger=self._log)
            command = "Set all"
        else:
            ok = _try_send_with_retries(cf, np_set_pixel, pixel_index, r, g, b, logger=self._log)
            command = f"Set pixel {pixel_index}"
        if ok:
            self._log(f"{command} to RGB ({r}, {g}, {b})")

    def show_colour(self) -> None:
        cf = self.cf
        if cf is None:
            self._log("Show requested without connection")
            return
        if _try_send_with_retries(cf, np_show, logger=self._log):
            self._log("Issued SHOW command")

    def clear_leds(self) -> None:
        cf = self.cf
        if cf is None:
            self._log("Clear requested without connection")
            return
        if _try_send_with_retries(cf, np_clear, logger=self._log):
            self._log("Cleared LEDs")
        if self.blinking:
            self.blinking = False

    def toggle_blink(self) -> None:
        cf = self.cf
        if cf is None:
            self._log("Blink requested without connection")
            return
        if self.blinking:
            if _try_send_with_retries(cf, np_stop_blink, logger=self._log):
                self._log("Stopped blinking")
                self.blinking = False
        else:
            r, g, b = self._clamp_rgb()
            if _try_send_with_retries(cf, np_set_all, r, g, b, logger=self._log):
                if _try_send_with_retries(cf, np_start_blink, logger=self._log):
                    self._log(f"Started blinking with RGB ({r}, {g}, {b})")
                    self.blinking = True

    def _clamp_rgb(self) -> tuple[int, int, int]:
        r = max(0, min(255, self.r_var.get()))
        g = max(0, min(255, self.g_var.get()))
        b = max(0, min(255, self.b_var.get()))
        self.r_var.set(r)
        self.g_var.set(g)
        self.b_var.set(b)
        return r, g, b

    def _set_status(self, text: str) -> None:
        self.status_var.set(text)

    def _log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        entry = f"[{timestamp}] {message}"
        print(f"[NeoPixel] {message}")
        self.log_list.insert(tk.END, entry)
        self.log_list.yview_moveto(1.0)

    def _on_close(self) -> None:
        self.disconnect()
        self.root.after(300, self.root.destroy)


def main() -> None:
    root = tk.Tk()
    app = NeoPixelApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
