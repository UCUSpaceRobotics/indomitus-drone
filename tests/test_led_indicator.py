"""Unit test for WS2812 LED Controller."""

import unittest
from src.utils.led_indicator import LEDController


class TestLEDController(unittest.TestCase):
    def setUp(self):
        self.config = {
            "enabled": True,
            "gpio_pin": 10,
            "spi_bus": 0,
            "spi_device": 0,
            "num_leds": 7,
            "brightness": 128,
            "dma_channel": 10,
            "blink_interval_s": 0.1,
            "colors": {
                "manual": [0, 255, 0],
                "autonomous": [255, 0, 0],
            },
        }
        self.led = LEDController(self.config)

    def tearDown(self):
        if self.led:
            self.led.close()

    def test_initial_state_is_green(self):
        self.assertEqual(self.led._current_color, (0, 255, 0))

    def test_set_autonomous_mode(self):
        self.led.set_autonomous_mode()
        self.assertEqual(self.led._current_color, (255, 0, 0))

    def test_set_manual_mode(self):
        self.led.set_autonomous_mode()
        self.assertEqual(self.led._current_color, (255, 0, 0))
        self.led.set_manual_mode()
        self.assertEqual(self.led._current_color, (0, 255, 0))

    def test_update_state_flag(self):
        self.led.update_state(is_autonomous=True)
        self.assertEqual(self.led._current_color, (255, 0, 0))
        self.led.update_state(is_autonomous=False)
        self.assertEqual(self.led._current_color, (0, 255, 0))

    def test_close(self):
        self.led.close()
        self.assertIsNone(self.led._current_color)
        self.assertTrue(self.led._thread is None or not self.led._thread.is_alive())

    def test_blinking_thread_running(self):
        self.assertIsNotNone(self.led._thread)
        self.assertTrue(self.led._thread.is_alive())

    def test_custom_blink_interval(self):
        cfg = dict(self.config)
        cfg["blink_rate_hz"] = 2.0
        led = LEDController(cfg)
        self.assertAlmostEqual(led.blink_interval, 0.25)
        led.close()

    def test_disabled_led(self):
        cfg = dict(self.config)
        cfg["enabled"] = False
        led = LEDController(cfg)
        self.assertFalse(led.enabled)
        self.assertIsNone(led._thread)
        led.set_autonomous_mode()
        led.close()


if __name__ == "__main__":
    unittest.main()

