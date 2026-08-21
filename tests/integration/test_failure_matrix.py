from src.mission.model import BroadState
from src.observations.model import Event, EventType, LandedState
from src.commands.types import FlightMode, LandHere
from tests.mission.helpers import coordinator, snapshot
from tests.mission.test_shutdown_policy import make_supervisor


def airborne_snapshot(now, ledger, **kwargs):
    return snapshot(
        now,
        ledger,
        mode=kwargs.get("mode", FlightMode.GUIDED),
        armed=True,
        landed=LandedState.IN_AIR,
    )


def test_dead_gateway_skips_impossible_landing_operation():
    mission, _, commands, ledger = coordinator()
    airborne = airborne_snapshot(0, ledger)
    mission._transition(BroadState.SEARCH, airborne, "test")
    mission.gateway.usable = False

    mission.update(airborne_snapshot(1, ledger))

    assert mission.phase is BroadState.AIRBORNE_FAULT
    assert commands.qsize() == 0


def test_event_inbox_overflow_conservatively_enters_land_here():
    supervisor, clock = make_supervisor()
    mission = supervisor.coordinator
    ledger = mission.gateway.ledger
    mission._transition(BroadState.SEARCH, airborne_snapshot(0, ledger), "test")
    for index in range(17):
        supervisor.inbox.append(
            Event(str(index), EventType.CAMERA, 0, None, "activity")
        )

    tick = supervisor.tick()

    assert tick.status.phase is BroadState.LAND_HERE
    envelope = mission.gateway._queue.get_nowait()
    assert isinstance(envelope.command, LandHere)
