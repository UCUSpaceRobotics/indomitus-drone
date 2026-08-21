import queue

from src.observations.event_inbox import EventInbox
from src.observations.model import LandedState, ObservationKey
from src.runtime.config import load_config
from src.runtime.supervisor import RuntimeSupervisor
from tests.mission.helpers import coordinator


class AliveProcess:
    def is_alive(self):
        return True

    def join(self, timeout=None):
        pass

    def terminate(self):
        pass


class StopEvent:
    def __init__(self):
        self.set_called = False

    def set(self):
        self.set_called = True


class Vision:
    def spin_once(self):
        pass

    def drain_events(self):
        return ()

    def shutdown(self):
        pass


class Rclpy:
    def shutdown(self):
        pass


def make_supervisor():
    mission, clock, _, _ = coordinator()
    config = load_config("config/mission_params.yaml")
    supervisor = RuntimeSupervisor(
        clock=clock,
        coordinator=mission,
        store=mission.store,
        inbox=EventInbox(16),
        telemetry_queue=queue.Queue(),
        result_queue=queue.Queue(),
        health_queue=queue.Queue(),
        comm_process=AliveProcess(),
        stop_event=StopEvent(),
        vision=Vision(),
        led=None,
        config=config,
        rclpy_module=Rclpy(),
    )
    return supervisor, clock


def test_shutdown_requires_fresh_landed_and_disarmed_evidence():
    supervisor, clock = make_supervisor()
    supervisor.store.put(
        ObservationKey.HEARTBEAT, {"armed": False}, 0.0
    )
    supervisor.store.put(ObservationKey.LANDED_STATE, LandedState.ON_GROUND, 0.0)
    assert supervisor.safe_shutdown_gate(1.0)
    assert not supervisor.safe_shutdown_gate(1.0001)


def test_force_shutdown_requires_audited_actor_and_reason():
    supervisor, _ = make_supervisor()
    supervisor.force_shutdown("operator", "bench emergency")
    assert supervisor._force_shutdown
    entry = supervisor.coordinator.journal.entries()[-1]
    assert entry.kind == "force-shutdown"
    assert entry.detail == {"actor": "operator", "reason": "bench emergency"}
