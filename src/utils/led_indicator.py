"""WS2812 5050 RGB LED Indicator Controller for Indomitus Drone.

Optimized for Raspberry Pi 5 using Hardware SPI (GPIO 10 / Physical Pin 19 - MOSI).
Also supports legacy rpi_ws281x DMA and mock mode for local testing.

Indicator States:
  - GREEN (Blinking): Manual control / standby
  - RED (Blinking): Autonomous execution
"""

from __future__ import annotations

import logging
import threading
import time
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

    Emits blinking signals (Green for manual/standby, Red for autonomous execution).

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

        # Blink timing configuration (seconds per half-cycle: ON duration / OFF duration)
        if "blink_rate_hz" in config:
            self.blink_interval = 0.5 / max(0.1, float(config["blink_rate_hz"]))
        elif "blink_interval_s" in config:
            self.blink_interval = float(config["blink_interval_s"])
        elif "blink_interval" in config:
            self.blink_interval = float(config["blink_interval"])
        else:
            self.blink_interval = 0.5  # Default: 0.5s ON / 0.5s OFF (1 Hz)

        # Colors from config (R, G, B)
        colors_cfg = config.get("colors", {})
        self.color_manual: Tuple[int, int, int] = tuple(colors_cfg.get("manual", [0, 255, 0]))
        self.color_autonomous: Tuple[int, int, int] = tuple(colors_cfg.get("autonomous", [255, 0, 0]))

        # State tracking and threading
        self._current_color: Tuple[int, int, int] | None = None
        self._driver_type: str = "mock"
        self._spi_driver: SPIWS2812Driver | None = None
        self._ws281x_strip = None

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._wake_event = threading.Event()
        self._thread: threading.Thread | None = None

        if not self.enabled:
            print("[LED] Disabled via configuration.")
            return

        # Attempt 1: SPI Driver (Primary for Raspberry Pi 5)
        if HAS_SPIDEV:
            try:
                self._spi_driver = SPIWS2812Driver(self.spi_bus, self.spi_device)
                self._driver_type = "spidev"
                print(f"[LED] Initialized WS2812 via SPI (/dev/spidev{self.spi_bus}.{self.spi_device}) on GPIO {self.gpio_pin} (Pi 5 ready).")
            except Exception as e:
                print(f"[LED] SPI initialization failed ({e}). Check if SPI is enabled in raspi-config.")

        # Attempt 2: Legacy rpi_ws281x Driver (Raspberry Pi 3/4)
        if self._driver_type == "mock" and HAS_RPI_WS281X:
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
            except Exception as e:
                print(f"[LED] rpi_ws281x initialization failed ({e}).")

        # Fallback: Mock Mode
        if self._driver_type == "mock":
            print("[LED] Hardware LED drivers not available. Running in mock mode.")

        # Start blinking worker thread and set initial mode
        self._start_thread()
        self.set_manual_mode(force=True)

    def _start_thread(self) -> None:
        """Start the background worker thread for blinking."""
        if self._thread is None or not self._thread.is_alive():
            self._stop_event.clear()
            self._wake_event.clear()
            self._thread = threading.Thread(
                target=self._blink_loop,
                daemon=True,
                name="LEDIndicatorThread",
            )
            self._thread.start()

    def _write_hardware(self, r: int, g: int, b: int) -> None:
        """Send RGB data to physical hardware driver with brightness scaling."""
        scale = self.brightness / 255.0
        r_adj = int(r * scale)
        g_adj = int(g * scale)
        b_adj = int(b * scale)

        try:
            if self._driver_type == "spidev" and self._spi_driver is not None:
                self._spi_driver.write_pixels(self.num_leds, r_adj, g_adj, b_adj)
            elif self._driver_type == "rpi_ws281x" and self._ws281x_strip is not None:
                c = Color(r_adj, g_adj, b_adj)
                for i in range(self.num_leds):
                    self._ws281x_strip.setPixelColor(i, c)
                self._ws281x_strip.show()
        except Exception as e:
            logger.debug(f"[LED] Hardware write failed: {e}")

    def _blink_loop(self) -> None:
        """Background thread worker to blink LEDs with the current active color."""
        while not self._stop_event.is_set():
            with self._lock:
                target_color = self._current_color

            if target_color is None or target_color == (0, 0, 0):
                # Turn off LEDs and wait until a new color is set or controller is stopped
                self._write_hardware(0, 0, 0)
                self._wake_event.wait(timeout=self.blink_interval)
                self._wake_event.clear()
                continue

            r, g, b = target_color

            # Phase 1: LEDs ON
            self._write_hardware(r, g, b)
            woken = self._wake_event.wait(timeout=self.blink_interval)
            if self._stop_event.is_set():
                break
            if woken:
                self._wake_event.clear()
                continue

            # Phase 2: LEDs OFF
            self._write_hardware(0, 0, 0)
            woken = self._wake_event.wait(timeout=self.blink_interval)
            if self._stop_event.is_set():
                break
            if woken:
                self._wake_event.clear()
                continue

    def set_color(self, r: int, g: int, b: int, force: bool = False) -> None:
        """Set active blinking RGB color.

        Uses state caching so calling set_color repeatedly with the same color
        does not disrupt the active blinking phase.
        """
        if not self.enabled:
            return

        color_tuple = (r, g, b)
        with self._lock:
            if not force and color_tuple == self._current_color:
                return  # Color hasn't changed — skip reset

            self._current_color = color_tuple

        print(f"[LED] Color updated: RGB({r}, {g}, {b}) [{self._driver_type}]")
        self._wake_event.set()

    def set_manual_mode(self, force: bool = False) -> None:
        """Set LED indicator to Green blinking (Manual control / Standby)."""
        r, g, b = self.color_manual
        self.set_color(r, g, b, force=force)

    def set_autonomous_mode(self, force: bool = False) -> None:
        """Set LED indicator to Red blinking (Autonomous execution)."""
        r, g, b = self.color_autonomous
        self.set_color(r, g, b, force=force)

    def update_state(self, is_autonomous: bool) -> None:
        """Update LED mode based on autonomy status flag."""
        if is_autonomous:
            self.set_autonomous_mode()
        else:
            self.set_manual_mode()

    def close(self) -> None:
        """Stop blinking, turn off all LEDs, and release hardware resources."""
        if not self.enabled:
            return

        self._stop_event.set()
        self._wake_event.set()

        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
            self._thread = None

        with self._lock:
            self._current_color = None

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

        print("[LED] Shut down (LEDs off).")

