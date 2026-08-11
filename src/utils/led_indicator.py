"""WS2812 5050 RGB LED Indicator Controller for Indomitus Drone.

Drives a 7-bit WS2812 LED ring/module connected to Raspberry Pi hardware DMA
(GPIO 18 / PWM0) to visually report drone flight state:
  - GREEN: Manual control / standby
  - RED: Autonomous execution

Optimized for low CPU overhead by using hardware DMA via rpi_ws281x and state caching.
"""

from __future__ import annotations

import logging
from typing import Tuple

logger = logging.getLogger(__name__)

# Try importing hardware library for Raspberry Pi
try:
    from rpi_ws281x import PixelStrip, Color, ws
    HAS_HARDWARE_LED = True
except (ImportError, RuntimeError):
    HAS_HARDWARE_LED = False


class LEDController:
    """Controls WS2812 5050 RGB LED hardware indicator.

    Args:
        config: Parsed 'led' dictionary from mission_params.yaml.
    """

    def __init__(self, config: dict):
        self.enabled = config.get("enabled", True)
        self.gpio_pin = config.get("gpio_pin", 18)
        self.num_leds = config.get("num_leds", 7)
        self.brightness = config.get("brightness", 128)
        self.dma_channel = config.get("dma_channel", 10)

        # Colors from config (R, G, B)
        colors_cfg = config.get("colors", {})
        self.color_manual: Tuple[int, int, int] = tuple(colors_cfg.get("manual", [0, 255, 0]))
        self.color_autonomous: Tuple[int, int, int] = tuple(colors_cfg.get("autonomous", [255, 0, 0]))

        # State cache to avoid unnecessary hardware updates
        self._current_color: Tuple[int, int, int] | None = None
        self._strip = None

        if not self.enabled:
            print("[LED] Disabled via configuration.")
            return

        if HAS_HARDWARE_LED:
            try:
                # Initialize rpi_ws281x PixelStrip (using PWM0 on GPIO 18)
                self._strip = PixelStrip(
                    self.num_leds,
                    self.gpio_pin,
                    800000,  # 800 kHz bit rate
                    self.dma_channel,
                    False,   # Invert signal
                    self.brightness,
                    0,       # Channel 0 (for GPIO 18 PWM)
                    ws.WS2811_STRIP_GRB,
                )
                self._strip.begin()
                print(f"[LED] Initialized WS2812 on GPIO {self.gpio_pin} ({self.num_leds} LEDs via DMA).")
                # Set initial state to Green (Manual control / Standby)
                self.set_manual_mode()
            except Exception as e:
                print(f"[LED] WARNING: Failed to initialize rpi_ws281x strip ({e}). Running in mock mode.")
                print("[LED] TROUBLESHOOTING:")
                print("[LED]  1. Make sure to run main.py with root privileges: sudo -E python3 main.py")
                print("[LED]  2. If using GPIO 18, disable Pi audio kernel module: echo 'blacklist snd_bcm2835' | sudo tee /etc/modprobe.d/snd-blacklist.conf")
                self._strip = None
        else:
            print("[LED] rpi_ws281x Python package is NOT installed. Running in mock mode.")
            print("[LED] Install it on Raspberry Pi via: pip install rpi_ws281x (or inside your venv).")
            # Set initial mock state to Green
            self.set_manual_mode()

    def set_color(self, r: int, g: int, b: int) -> None:
        """Set all LEDs to the specified RGB color.

        Uses state caching to prevent redundant DMA hardware transfers.
        """
        color_tuple = (r, g, b)
        if color_tuple == self._current_color:
            return  # Color hasn't changed — skip hardware write for 0 CPU load.

        self._current_color = color_tuple

        if self._strip is not None:
            c = Color(r, g, b)
            for i in range(self.num_leds):
                self._strip.setPixelColor(i, c)
            self._strip.show()

        print(f"[LED] Color updated: RGB({r}, {g}, {b})")

    def set_manual_mode(self) -> None:
        """Set LED indicator to Green (Manual control / Standby)."""
        r, g, b = self.color_manual
        self.set_color(r, g, b)

    def set_autonomous_mode(self) -> None:
        """Set LED indicator to Red (Autonomy execution)."""
        r, g, b = self.color_autonomous
        self.set_color(r, g, b)

    def update_state(self, is_autonomous: bool) -> None:
        """Update LED mode based on autonomy status flag."""
        if is_autonomous:
            self.set_autonomous_mode()
        else:
            self.set_manual_mode()

    def close(self) -> None:
        """Turn off all LEDs and release hardware resources."""
        if self._strip is not None:
            try:
                for i in range(self.num_leds):
                    self._strip.setPixelColor(i, Color(0, 0, 0))
                self._strip.show()
            except Exception:
                pass
        self._current_color = None
        print("[LED] Shut down (LEDs off).")
