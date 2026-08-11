"""Unit test for WS2812 LED Controller."""

import unittest
from src.utils.led_indicator import LEDController


class TestLEDController(unittest.TestCase):
    def setUp(self):
        self.config = {
            "enabled": True,
            "gpio_pin": 18,
            "num_leds": 7,
            "brightness": 128,
            "dma_channel": 10,
            "colors": {
                "manual": [0, 255, 0],
                "autonomous": [255, 0, 0],
            },
        }
        self.led = LEDController(self.config)

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


if __name__ == "__main__":
    unittest.main()
