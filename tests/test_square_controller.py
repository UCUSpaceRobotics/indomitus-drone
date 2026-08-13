"""Unit tests for SquareFlightController logic, square traversal, and safety failsafes."""

import queue
import time
import unittest

from test_square_flight import SquareFlightController, SquareFlightState


class TestSquareFlightController(unittest.TestCase):
    def setUp(self):
        self.cmd_q = queue.Queue()
        self.telem_q = queue.Queue()
        self.config = {
            "serial": {"port": "/dev/ttyAMA0", "baudrate": 921600},
            "timeouts": {"takeoff_s": 5.0, "landing_s": 5.0},
            "led": {"enabled": False},
        }
        self.controller = SquareFlightController(
            command_queue=self.cmd_q,
            telemetry_queue=self.telem_q,
            config=self.config,
            led_indicator=None,
            target_altitude_m=1.0,
            square_side_m=1.0,
            settle_duration_s=0.01,  # Fast settle for test execution
        )

    def test_idle_to_takeoff_transition(self):
        """Verify controller stays in IDLE until connected and EKF is healthy."""
        self.controller.update()
        self.assertEqual(self.controller.state, SquareFlightState.IDLE)

        # Feed healthy telemetry
        self.controller.telem = {
            "connected": True,
            "ekf_healthy": True,
            "armed": False,
            "mode": "LOITER",
            "pos_z_m": 0.0,
        }
        self.controller.update()
        self.assertEqual(self.controller.state, SquareFlightState.TAKEOFF)

    def test_full_square_flight_sequence(self):
        """Simulate the full takeoff -> forward -> right -> back -> left -> land sequence."""
        # 1. Takeoff phase
        self.controller.state = SquareFlightState.TAKEOFF
        self.controller._takeoff_phase = 4  # Climbing phase
        self.controller.telem = {
            "connected": True,
            "ekf_healthy": True,
            "armed": True,
            "mode": "GUIDED",
            "pos_x_m": 0.0,
            "pos_y_m": 0.0,
            "pos_z_m": -0.95,  # 0.95m altitude
        }
        self.controller.update()
        self.assertEqual(self.controller.state, SquareFlightState.HOVER_SETTLE)

        # 2. Settle -> Forward
        time.sleep(0.02)
        self.controller.update()
        self.assertEqual(self.controller.state, SquareFlightState.MOVE_FORWARD)

        # Update once in MOVE_FORWARD to dispatch the movement command
        self.controller.update()
        self.assertFalse(self.cmd_q.empty())
        cmd = self.cmd_q.get_nowait()
        self.assertEqual(cmd["action"], "move_local_pos")
        self.assertEqual(cmd["dx"], 1.0)
        self.assertEqual(cmd["dy"], 0.0)

        # 3. Complete Forward Leg (reached x=1.0) -> Right
        self.controller.telem["pos_x_m"] = 1.0
        self.controller.update()
        time.sleep(0.02)
        self.controller.update()
        self.assertEqual(self.controller.state, SquareFlightState.MOVE_RIGHT)

        self.controller.update()
        cmd = self.cmd_q.get_nowait()
        self.assertEqual(cmd["action"], "move_local_pos")
        self.assertEqual(cmd["dx"], 0.0)
        self.assertEqual(cmd["dy"], 1.0)

        # 4. Complete Right Leg (reached y=1.0) -> Back
        self.controller.telem["pos_y_m"] = 1.0
        self.controller.update()
        time.sleep(0.02)
        self.controller.update()
        self.assertEqual(self.controller.state, SquareFlightState.MOVE_BACK)

        self.controller.update()
        cmd = self.cmd_q.get_nowait()
        self.assertEqual(cmd["action"], "move_local_pos")
        self.assertEqual(cmd["dx"], -1.0)
        self.assertEqual(cmd["dy"], 0.0)

        # 5. Complete Back Leg (reached x=0.0) -> Left
        self.controller.telem["pos_x_m"] = 0.0
        self.controller.update()
        time.sleep(0.02)
        self.controller.update()
        self.assertEqual(self.controller.state, SquareFlightState.MOVE_LEFT)

        self.controller.update()
        cmd = self.cmd_q.get_nowait()
        self.assertEqual(cmd["action"], "move_local_pos")
        self.assertEqual(cmd["dx"], 0.0)
        self.assertEqual(cmd["dy"], -1.0)

        # 6. Complete Left Leg (reached y=0.0) -> Land
        self.controller.telem["pos_y_m"] = 0.0
        self.controller.update()
        time.sleep(0.02)
        self.controller.update()
        self.assertEqual(self.controller.state, SquareFlightState.LAND)

        # 7. Landing touchdown
        self.controller.update()
        cmd = self.cmd_q.get_nowait()
        self.assertEqual(cmd["action"], "set_mode")
        self.assertEqual(cmd["mode"], "LAND")

        self.controller.telem["pos_z_m"] = -0.05
        self.controller.telem["armed"] = False
        self.controller.update()
        self.assertEqual(self.controller.state, SquareFlightState.COMPLETE)

    def test_manual_override_detection(self):
        """Verify pilot switching mode away from GUIDED triggers MANUAL_OVERRIDE."""
        self.controller.state = SquareFlightState.MOVE_FORWARD
        self.controller.telem = {
            "connected": True,
            "ekf_healthy": True,
            "armed": True,
            "mode": "STABILIZE",  # Pilot flipped flight mode switch
            "pos_x_m": 0.5,
            "pos_y_m": 0.0,
            "pos_z_m": -1.0,
        }
        self.controller.update()
        self.assertEqual(self.controller.state, SquareFlightState.MANUAL_OVERRIDE)


if __name__ == "__main__":
    unittest.main()
