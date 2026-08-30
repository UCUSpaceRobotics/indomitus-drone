"""Unit tests for state machine with Simulink synchronization."""

import queue
import unittest

from src.navigation.state_machine import FlightState, MissionController


class DummyVisionBridge:
    def __init__(self):
        self.simulink_state = None
        self.published_telemetry = []
        self.latest_target = None

    def spin_once(self):
        pass

    def get_simulink_state(self):
        return self.simulink_state

    def get_latest_target(self):
        return self.latest_target

    def publish_telemetry(self, **kwargs):
        self.published_telemetry.append(kwargs)


class TestSimulinkStateMachine(unittest.TestCase):
    def setUp(self):
        self.cmd_q = queue.Queue()
        self.telem_q = queue.Queue()
        self.vision = DummyVisionBridge()
        self.config = {
            "flight": {"takeoff_altitude_m": 1.5},
            "markers": {"landing_target_id": 102},
            "timeouts": {
                "takeoff_s": 30.0,
                "search_sweep_s": 320.0,
                "landing_s": 30.0,
                "alignment_s": 10.0,
            },
        }
        self.mission = MissionController(
            command_queue=self.cmd_q,
            telemetry_queue=self.telem_q,
            vision_bridge=self.vision,
            config=self.config,
            led_indicator=None,
        )

    def test_initial_state(self):
        self.assertEqual(self.mission.state, FlightState.IDLE)

    def test_telemetry_published_on_tick(self):
        self.telem_q.put({
            "pos_x_m": 0.1,
            "pos_y_m": 0.2,
            "pos_z_m": -1.5,
            "armed": True,
            "ekf_healthy": True,
            "connected": True,
            "mode": "GUIDED",
        })
        self.mission.update()
        self.assertTrue(len(self.vision.published_telemetry) > 0)
        last_telem = self.vision.published_telemetry[-1]
        self.assertAlmostEqual(last_telem["alt"], 1.5)
        self.assertTrue(last_telem["is_armed"])
        self.assertTrue(last_telem["ekf_healthy"])
        self.assertTrue(last_telem["connected"])

    def test_simulink_state_sync_takeoff_to_search(self):
        self.mission.state = FlightState.TAKEOFF
        self.telem_q.put({
            "pos_x_m": 0.0,
            "pos_y_m": 0.0,
            "pos_z_m": -1.3,
            "armed": True,
            "ekf_healthy": True,
            "connected": True,
            "mode": "GUIDED",
        })
        self.vision.simulink_state = 2
        self.mission.update()
        self.assertEqual(self.mission.state, FlightState.SEARCH)

    def test_geofence_failsafe(self):
        self.mission.state = FlightState.SEARCH
        self.telem_q.put({
            "pos_x_m": 6.0,
            "pos_y_m": 6.0,
            "pos_z_m": -1.5,
            "armed": True,
            "ekf_healthy": True,
            "connected": True,
            "mode": "GUIDED",
        })
        self.mission.update()
        self.assertEqual(self.mission.state, FlightState.DESCEND)
        cmds = []
        while not self.cmd_q.empty():
            cmds.append(self.cmd_q.get()["action"])
        self.assertIn("set_mode", cmds)

    def test_touchdown_to_complete(self):
        self.mission.state = FlightState.DESCEND
        self.vision.simulink_state = 3  # Simulink is in DESCEND
        self.telem_q.put({
            "pos_x_m": 0.0,
            "pos_y_m": 0.0,
            "pos_z_m": -0.1,  # alt = 0.1m < 0.25m
            "armed": False,   # disarmed
            "ekf_healthy": True,
            "connected": True,
            "mode": "LAND",
        })
        self.mission.update()
        self.assertEqual(self.mission.state, FlightState.COMPLETE)


if __name__ == "__main__":
    unittest.main()
