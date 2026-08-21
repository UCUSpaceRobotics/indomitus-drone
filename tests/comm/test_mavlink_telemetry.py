from src.comm.messages import TelemetryPacket
from src.observations.model import LandedState, ObservationKey
from tests.mission.test_shutdown_policy import make_supervisor


def test_extended_landed_state_and_disarm_feed_shutdown_gate():
    supervisor, clock = make_supervisor()
    clock.value = 1.0
    supervisor.telemetry_queue.put_nowait(
        TelemetryPacket(
            {
                "mode": "LAND",
                "armed": False,
                "last_heartbeat_time": 1.0,
                "landed_state": "on-ground",
                "last_landed_state_time": 1.0,
            },
            1.0,
        )
    )
    supervisor._drain_telemetry()
    assert supervisor.store.latest(ObservationKey.LANDED_STATE).value is LandedState.ON_GROUND
    assert supervisor.safe_shutdown_gate(1.0)
