"""WS2812 5050 RGB LED Indicator Controller for Indomitus Drone.

Optimized for Raspberry Pi 5 using Hardware SPI (GPIO 10 / Physical Pin 19 - MOSI).
Also supports legacy rpi_ws281x DMA and mock mode for local testing.

Indicator States:
  - GREEN: Manual control / standby
  - RED: Autonomous execution
"""

from __future__ import annotations

import logging
from typing import Tuple

logger = logging.getLogger(__name__)

# Try importing SPI driver (Recommended for Raspberry Pi 5)
try:
    import spidev
    HAS_SPIDEV = True
except ImportError:
    HAS_SPIDEV = False

# Try importing legacy rpi_ws281x driver (Raspberry Pi 3/4)
try:
    from rpi_ws281x import PixelStrip, Color, ws
    HAS_RPI_WS281X = True
except (ImportError, RuntimeError):
    HAS_RPI_WS281X = False


class SPIWS2812Driver:
    """Hardware SPI driver for WS2812 on Raspberry Pi 5 (/dev/spidev0.0)."""

    def __init__(self, bus: int = 0, device: int = 0, speed_hz: int = 6400000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0

        # Send initial reset pulse to clear WS2812 shift register
        reset_buf = b"\x00" * 100
        self._write_raw(reset_buf)

    def _write_raw(self, data: bytes | bytearray) -> None:
        """Write raw byte buffer to SPI hardware."""
        if hasattr(self.spi, "writebytes2"):
            self.spi.writebytes2(data)
        elif hasattr(self.spi, "writebytes"):
            self.spi.writebytes(list(data))
        else:
            self.spi.xfer2(list(data))

    def write_pixels(self, num_leds: int, r: int, g: int, b: int) -> None:
        """Serialize WS2812 GRB bits to SPI bytes at 6.4 MHz.

        Bit '0': 0xC0 (2 bits HIGH ~312ns, 6 bits LOW ~938ns)
        Bit '1': 0xFC (6 bits HIGH ~938ns, 2 bits LOW ~312ns)
        """
        tx_buf = bytearray()
        for _ in range(num_leds):
            for color_val in (g, r, b):  # WS2812 GRB order
                for bit_idx in range(7, -1, -1):
                    bit = (color_val >> bit_idx) & 1
                    tx_buf.append(0xFC if bit else 0xC0)

        # Append latch reset pulse (>50us LOW)
        tx_buf.extend(b"\x00" * 80)
        self._write_raw(tx_buf)

    def close(self) -> None:
        try:
            # Send black (all off) before closing
            reset_buf = b"\x00" * 100
            self._write_raw(reset_buf)
            self.spi.close()
        except Exception:
            pass


class LEDController:
    """Controls WS2812 5050 RGB LED hardware indicator for Raspberry Pi 5 & 4.

    Args:
        config: Parsed 'led' dictionary from mission_params.yaml.
    """

    def __init__(self, config: dict):
        self.enabled = config.get("enabled", True)
        self.gpio_pin = config.get("gpio_pin", 10)  # GPIO 10 for SPI MOSI
        self.num_leds = config.get("num_leds", 7)
        self.brightness = config.get("brightness", 128)
        self.spi_bus = config.get("spi_bus", 0)
        self.spi_device = config.get("spi_device", 0)
        self.dma_channel = config.get("dma_channel", 10)

        # Colors from config (R, G, B)
        colors_cfg = config.get("colors", {})
        self.color_manual: Tuple[int, int, int] = tuple(colors_cfg.get("manual", [0, 255, 0]))
        self.color_autonomous: Tuple[int, int, int] = tuple(colors_cfg.get("autonomous", [255, 0, 0]))

        # State cache to avoid unnecessary hardware updates
        self._current_color: Tuple[int, int, int] | None = None
        self._driver_type: str = "mock"
        self._spi_driver: SPIWS2812Driver | None = None
        self._ws281x_strip = None

        if not self.enabled:
            print("[LED] Disabled via configuration.")
            return

        # Attempt 1: SPI Driver (Primary for Raspberry Pi 5)
        if HAS_SPIDEV:
            try:
                self._spi_driver = SPIWS2812Driver(self.spi_bus, self.spi_device)
                self._driver_type = "spidev"
                print(f"[LED] Initialized WS2812 via SPI (/dev/spidev{self.spi_bus}.{self.spi_device}) on GPIO {self.gpio_pin} (Pi 5 ready).")
                self.set_manual_mode(force=True)
                return
            except Exception as e:
                print(f"[LED] SPI initialization failed ({e}). Check if SPI is enabled in raspi-config.")

        # Attempt 2: Legacy rpi_ws281x Driver (Raspberry Pi 3/4)
        if HAS_RPI_WS281X:
            try:
                self._ws281x_strip = PixelStrip(
                    self.num_leds,
                    self.gpio_pin,
                    800000,
                    self.dma_channel,
                    False,
                    self.brightness,
                    0,
                    ws.WS2811_STRIP_GRB,
                )
                self._ws281x_strip.begin()
                self._driver_type = "rpi_ws281x"
                print(f"[LED] Initialized WS2812 via rpi_ws281x on GPIO {self.gpio_pin}.")
                self.set_manual_mode(force=True)
                return
            except Exception as e:
                print(f"[LED] rpi_ws281x initialization failed ({e}).")

        # Fallback: Mock Mode
        print("[LED] Hardware LED drivers not available. Running in mock mode.")
        self.set_manual_mode(force=True)

    def set_color(self, r: int, g: int, b: int, force: bool = False) -> None:
        """Set all LEDs to the specified RGB color with brightness scaling.

        Uses state caching to prevent redundant SPI/DMA hardware transfers.
        """
        color_tuple = (r, g, b)
        if not force and color_tuple == self._current_color:
            return  # Color hasn't changed — skip hardware write for 0 CPU load.

        self._current_color = color_tuple

        # Scale brightness (0-255)
        scale = self.brightness / 255.0
        r_adj = int(r * scale)
        g_adj = int(g * scale)
        b_adj = int(b * scale)

        if self._driver_type == "spidev" and self._spi_driver is not None:
            self._spi_driver.write_pixels(self.num_leds, r_adj, g_adj, b_adj)

        elif self._driver_type == "rpi_ws281x" and self._ws281x_strip is not None:
            c = Color(r_adj, g_adj, b_adj)
            for i in range(self.num_leds):
                self._ws281x_strip.setPixelColor(i, c)
            self._ws281x_strip.show()

        print(f"[LED] Color updated: RGB({r}, {g}, {b}) [{self._driver_type}]")

    def set_manual_mode(self, force: bool = False) -> None:
        """Set LED indicator to Green (Manual control / Standby)."""
        r, g, b = self.color_manual
        self.set_color(r, g, b, force=force)

    def set_autonomous_mode(self, force: bool = False) -> None:
        """Set LED indicator to Red (Autonomy execution)."""
        r, g, b = self.color_autonomous
        self.set_color(r, g, b, force=force)

    def update_state(self, is_autonomous: bool) -> None:
        """Update LED mode based on autonomy status flag."""
        if is_autonomous:
            self.set_autonomous_mode()
        else:
            self.set_manual_mode()

    def close(self) -> None:
        """Turn off all LEDs and release hardware resources."""
        if self._driver_type == "spidev" and self._spi_driver is not None:
            try:
                self._spi_driver.write_pixels(self.num_leds, 0, 0, 0)
                self._spi_driver.close()
            except Exception:
                pass
        elif self._driver_type == "rpi_ws281x" and self._ws281x_strip is not None:
            try:
                for i in range(self.num_leds):
                    self._ws281x_strip.setPixelColor(i, Color(0, 0, 0))
                self._ws281x_strip.show()
            except Exception:
                pass

        self._current_color = None
        print("[LED] Shut down (LEDs off).")
