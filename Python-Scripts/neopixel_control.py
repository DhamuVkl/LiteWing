"""
LiteWing NeoPixel LED Control

Control the NeoPixel LEDs on your LiteWing drone over CRTP/WiFi.
"""

import time
import cflib.crtp
from cflib.crazyflie import Crazyflie

# CRTP ports and channels for NeoPixel control
CRTP_PORT_NEOPIXEL = 0x09
NEOPIXEL_CHANNEL_SET_PIXEL = 0x00
NEOPIXEL_CHANNEL_SHOW = 0x01
NEOPIXEL_CHANNEL_CLEAR = 0x02
NEOPIXEL_CHANNEL_BLINK = 0x03


class NeoPixelController:
    def __init__(self, uri_or_cf):
        """Initialize the NeoPixel controller.

        Accepts either a URI string (e.g. "udp://192.168.43.42") or an
        already-created Crazyflie instance. If a Crazyflie instance is
        provided the controller will use the existing link and will NOT
        attempt to open/close it (avoids binding the UDP socket twice).

        Args:
            uri_or_cf: URI string or Crazyflie instance
        """
        # If the caller passed an existing Crazyflie instance, use it.
        if isinstance(uri_or_cf, Crazyflie) or hasattr(uri_or_cf, "link"):
            self._cf = uri_or_cf
            self._uri = None
            self._owns_link = False
        else:
            # Treat as URI string and create our own Crazyflie instance
            self._cf = Crazyflie(rw_cache="./cache")
            self._uri = uri_or_cf
            self._owns_link = True

    def connect(self):
        """Connect to the drone."""
        if not self._owns_link:
            # Already using an external Crazyflie instance with an open link
            # (e.g. passed from SyncCrazyflie). Nothing to do.
            print("NeoPixelController: using existing Crazyflie link (no open needed)")
            return

        print(f"Connecting to {self._uri}...")
        self._cf.open_link(self._uri)
        # Give the link a moment to establish
        # Some cflib versions need a short delay before the link object is usable
        import time as _time

        _time.sleep(1)

    def disconnect(self):
        """Disconnect from the drone."""
        # Only close the link if we opened it ourselves
        if self._owns_link:
            try:
                self._cf.close_link()
            except Exception:
                pass

    def set_pixel(self, index, r, g, b):
        """Set the color of a single pixel.

        Args:
            index: Pixel index (0-based)
            r, g, b: RGB color values (0-255)
        """
        self._send_crtp(
            CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_SET_PIXEL, bytes([index, r, g, b])
        )

    def show(self):
        """Show the current pixel colors."""

        self._send_crtp(CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_SHOW, b"")

    def clear(self):
        """Clear all pixels (turn off)."""

        self._send_crtp(CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_CLEAR, b"")

    def start_blink(self, on_ms=500, off_ms=500):
        """Start blinking the current colors.

        Args:
            on_ms: On time in milliseconds
            off_ms: Off time in milliseconds
        """
        data = bytes(
            [
                1,
                (on_ms >> 8) & 0xFF,
                on_ms & 0xFF,
                (off_ms >> 8) & 0xFF,
                off_ms & 0xFF,
            ]
        )
        self._send_crtp(CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_BLINK, data)

    def stop_blink(self):
        """Stop blinking."""
        data = bytes([0, 0, 0, 0, 0])
        self._send_crtp(CRTP_PORT_NEOPIXEL, NEOPIXEL_CHANNEL_BLINK, data)

    def _send_crtp(self, port, channel, payload: bytes):
        """Low-level helper to send a CRTP packet, with fallbacks for different cflib versions."""
        header = ((port & 0x0F) << 4) | (channel & 0x0F)

        # Build a packet-like object expected by different cflib implementations.
        class _PacketObj:
            def __init__(self, header, data: bytes):
                self.header = header
                self.data = data
                # Some drivers expect a tuple of ints named 'datat'
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

        # Try several send methods depending on cflib version
        # 1) Crazyflie.send_packet (some versions)
        try:
            send_fn = getattr(self._cf, "send_packet", None)
            if callable(send_fn):
                # print debug and use it
                print("CRTP: trying Crazyflie.send_packet")
                try:
                    send_fn(pkt_obj)
                    print("CRTP: Crazyflie.send_packet succeeded")
                    return
                except Exception as e:
                    import traceback

                    print("CRTP: Crazyflie.send_packet failed:", e)
                    print(traceback.format_exc())
                    # fall through to next method
        except Exception:
            pass

        # 2) low-level link object: _link.sendPacket or _link.send_packet
        try:
            link = getattr(self._cf, "_link", None) or getattr(self._cf, "link", None)
            if link is not None:
                if hasattr(link, "sendPacket"):
                    print("CRTP: trying link.sendPacket")
                    try:
                        link.sendPacket(pkt_obj)
                        print("CRTP: link.sendPacket succeeded")
                        return
                    except Exception as e:
                        import traceback

                        print("CRTP: link.sendPacket failed:", e)
                        print(traceback.format_exc())
                if hasattr(link, "send_packet"):
                    print("CRTP: trying link.send_packet")
                    try:
                        link.send_packet(pkt_obj)
                        print("CRTP: link.send_packet succeeded")
                        return
                    except Exception as e:
                        import traceback

                        print("CRTP: link.send_packet failed:", e)
                        print(traceback.format_exc())
        except Exception:
            pass

        # 3) cflib.crtp: try crtp.send_packet if available
        try:
            import cflib.crtp as _crtp

            sendp = getattr(_crtp, "send_packet", None)
            if callable(sendp):
                print("CRTP: trying cflib.crtp.send_packet")
                try:
                    # try passing packet object first
                    sendp(pkt_obj)
                    print("CRTP: cflib.crtp.send_packet succeeded (obj)")
                    return
                except Exception:
                    # fallback to raw bytes
                    try:
                        sendp(bytes([pkt_obj.header]) + pkt_obj.data)
                        print("CRTP: cflib.crtp.send_packet succeeded (raw)")
                        return
                    except Exception as e:
                        import traceback

                        print("CRTP: cflib.crtp.send_packet failed:", e)
                        print(traceback.format_exc())
        except Exception:
            pass

        raise RuntimeError(
            "Unable to send CRTP packet: no send method available on Crazyflie instance"
        )


def demo():
    """Run a demo showing various NeoPixel control options."""
    # Initialize drivers
    cflib.crtp.init_drivers()

    # Connect to drone
    neo = NeoPixelController("udp://192.168.43.42")  # Update IP as needed
    neo.connect()

    try:
        print("Running NeoPixel demo...")

        # Clear all pixels
        print("Clearing pixels...")
        neo.clear()
        time.sleep(1)

        # Set individual pixels
        print("Setting pixels...")
        neo.set_pixel(0, 255, 0, 0)  # Red
        neo.set_pixel(1, 0, 255, 0)  # Green
        neo.set_pixel(2, 0, 0, 255)  # Blue
        neo.set_pixel(3, 255, 255, 255)  # White
        neo.show()
        time.sleep(2)

        # Start blinking
        print("Starting blink...")
        neo.start_blink(500, 500)  # 500ms on, 500ms off
        time.sleep(4)

        # Stop blinking
        print("Stopping blink...")
        neo.stop_blink()
        time.sleep(1)

        # Clear before exit
        neo.clear()

    finally:
        # Always disconnect properly
        neo.disconnect()


if __name__ == "__main__":
    demo()
