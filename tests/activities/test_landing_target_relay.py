from src.activities.landing_target_relay import LandingTargetRelay
from src.commands.gateway import CommandGateway
from src.commands.ledger import OperationLedger
from src.observations.model import CameraObservation, ObservationKey, TargetDownDistance
from src.observations.store import ObservationStore
from tests.fakes.clock import FakeClock
from tests.fakes.queues import SpyQueue


def test_relay_requires_fresh_positive_z_and_deduplicates_observations():
    clock = FakeClock(1.0)
    queue = SpyQueue()
    gateway = CommandGateway(queue, OperationLedger(32), clock)
    relay = LandingTargetRelay(
        gateway, "m", camera_freshness_s=0.5, distance_freshness_s=0.25
    )
    relay.start()
    store = ObservationStore()
    event = CameraObservation("1", 102, 0.2, -0.3, 1.0, 1.0)

    assert not relay.handle(event, store, 1.0)
    assert queue.calls == 0
    store.put(
        ObservationKey.TARGET_DOWN_DISTANCE,
        TargetDownDistance(2.0, 1.0),
        1.0,
    )
    assert relay.handle(event, store, 1.0)
    assert not relay.handle(event, store, 1.0)
    assert queue.calls == 1
    command = queue.values[0].command
    assert command.forward_m == -0.3
    assert command.right_m == 0.2
    assert command.down_m == 2.0


def test_relay_ignores_non_target_and_stale_events():
    clock = FakeClock(2.0)
    queue = SpyQueue()
    relay = LandingTargetRelay(
        CommandGateway(queue, OperationLedger(32), clock),
        "m",
        camera_freshness_s=0.5,
        distance_freshness_s=0.5,
    )
    relay.start()
    store = ObservationStore()
    store.put(ObservationKey.TARGET_DOWN_DISTANCE, 1.0, 2.0)
    assert not relay.handle(CameraObservation("a", 101, 0, 0, 2, 2), store, 2)
    assert not relay.handle(CameraObservation("b", 102, 0, 0, 1, 1), store, 2)
    assert queue.calls == 0
