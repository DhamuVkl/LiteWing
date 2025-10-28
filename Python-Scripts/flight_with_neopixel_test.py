"""
Flight + NeoPixel CRTP test for LiteWing using direct CRTP commands

This script:
- Initializes cflib CRTP drivers
- Opens a single Crazyflie link to the drone
- Uses direct CRTP commands for NeoPixel control
- Runs a short blink demo, then a simple motor on/off test
- Cleans up (stops blinking, clears LEDs, stops motors, closes link)

Adjust DRONE_URI as needed.
"""

import time
import struct
import cflib.crtp
from cflib.crazyflie import Crazyflie


# Inline robust CRTP send + NeoPixel helpers (copied/adapted from neopixel_control.py)
def _send_crtp_with_fallback(cf, port, channel, payload: bytes):
    """Send a CRTP packet using several fallbacks so this works across cflib versions.

    Args:
        cf: Crazyflie instance (already connected or with open link)
        port: CRTP port (e.g. 0x09 for NeoPixels)
        channel: NeoPixel channel (0=SET_PIXEL,1=SHOW,2=CLEAR,3=BLINK)
        payload: bytes payload
    """
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
                # fall through to other methods
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
        [
            1,
            (on_ms >> 8) & 0xFF,
            on_ms & 0xFF,
            (off_ms >> 8) & 0xFF,
            off_ms & 0xFF,
        ]
    )
    _send_crtp_with_fallback(cf, CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_BLINK, data)


def np_stop_blink(cf):
    data = bytes([0, 0, 0, 0, 0])
    _send_crtp_with_fallback(cf, CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_BLINK, data)


# URI for your LiteWing drone
DRONE_URI = "udp://192.168.43.42"


def main():
    # Initialize CRTP drivers (required by some cflib versions)
    cflib.crtp.init_drivers()

    print("Flight + NeoPixel test starting")

    cf = Crazyflie(rw_cache="./cache")

    try:
        print(f"Connecting to {DRONE_URI}...")
        cf.open_link(DRONE_URI)
        # Give the link a moment to stabilize
        time.sleep(1.0)

        # NeoPixel demo using inline helpers (reuses same Crazyflie link)
        try:
            print("Setting pixels and showing...")

            print("Setting LED 0 to Red...")
            np_set_pixel(cf, 0, 255, 0, 0)
            time.sleep(0.1)

            print("Setting LED 1 to Green...")
            np_set_pixel(cf, 1, 0, 255, 0)
            time.sleep(0.1)

            print("Setting LED 2 to Blue...")
            np_set_pixel(cf, 2, 0, 0, 255)
            time.sleep(0.1)

            print("Setting LED 3 to White...")
            np_set_pixel(cf, 3, 255, 255, 255)
            time.sleep(0.1)

            np_show(cf)
            time.sleep(1)

            print("Start blinking (500ms/500ms) for 4s")
            np_start_blink(cf, 500, 500)
            time.sleep(4)

            print("Stop blinking and clear")
            np_stop_blink(cf)
            np_clear(cf)
            time.sleep(0.5)
        except Exception as e:
            print(f"NeoPixel demo error: {e}")

        # --- Simple motor on/off test (same as your example) ---
        print("Sending zero setpoint to unlock safety...")
        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        roll = 0.0
        pitch = 0.0
        yaw = 0
        thrust = 10000  # Thrust value is 10000 minimum and 60000 maximum

        print("Starting motors at minimum thrust for 1s...")
        cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        time.sleep(1.0)

        print("Stopping motors")
        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

    except Exception as ex:
        print(f"Exception during test: {ex}")

    finally:
        # Ensure LEDs are off and link closed
        try:
            # Ensure NeoPixel cleared (use the inline helpers which reuse the link)
            try:
                np_stop_blink(cf)
            except Exception:
                pass
            try:
                np_clear(cf)
            except Exception:
                pass
        except Exception:
            pass

        try:
            cf.close_link()
        except Exception:
            pass

        print("Test complete")


if __name__ == "__main__":
    main()
