from collections import Counter

from src.commands.ledger import OperationStatus
from src.commands.types import (
    Arm,
    MoveToLocalNed,
    PrecisionLand,
    SetMode,
    Takeoff,
    FlightMode,
)
from src.mission.model import BroadState
from src.navigation.ned import LocalNed
from src.observations.model import LandedState
from tests.mission.helpers import coordinator, snapshot


def dispatch(ledger, operation_id, now, acknowledge=False):
    ledger.transition(operation_id, OperationStatus.DISPATCHED, now)
    if acknowledge:
        ledger.transition(operation_id, OperationStatus.ACKNOWLEDGED, now)


def test_fake_full_single_sortie_has_one_command_per_semantic_operation():
    mission, clock, commands, ledger = coordinator(route_legs=1)

    mission.update(snapshot(0, ledger))
    mission.update(snapshot(0, ledger))
    dispatch(ledger, mission.state.active_operation_id, 0)

    mission.update(snapshot(0.1, ledger, mode=FlightMode.LOITER))
    dispatch(ledger, mission.state.active_operation_id, 0.1, acknowledge=True)
    mission.update(snapshot(0.2, ledger, mode=FlightMode.LOITER, armed=True))
    dispatch(ledger, mission.state.active_operation_id, 0.2)
    mission.update(snapshot(0.3, ledger, mode=FlightMode.GUIDED, armed=True))
    dispatch(ledger, mission.state.active_operation_id, 0.3, acknowledge=True)

    mission.update(
        snapshot(
            1.0,
            ledger,
            mode=FlightMode.GUIDED,
            armed=True,
            position=LocalNed(0, 0, -1.9),
            landed=LandedState.IN_AIR,
        )
    )
    assert mission.phase is BroadState.SEARCH

    mission.update(
        snapshot(
            1.1,
            ledger,
            mode=FlightMode.GUIDED,
            armed=True,
            position=LocalNed(0, 0, -1.9),
            landed=LandedState.IN_AIR,
        )
    )
    mission.update(
        snapshot(
            1.2,
            ledger,
            mode=FlightMode.GUIDED,
            armed=True,
            position=LocalNed(0, 0, -1.9),
            landed=LandedState.IN_AIR,
        )
    )
    dispatch(ledger, mission.state.active_operation_id, 1.2)
    mission.update(
        snapshot(
            2.0,
            ledger,
            mode=FlightMode.GUIDED,
            armed=True,
            position=LocalNed(1, 0, -1.9),
            landed=LandedState.IN_AIR,
        )
    )
    mission.update(
        snapshot(
            3.0,
            ledger,
            mode=FlightMode.GUIDED,
            armed=True,
            position=LocalNed(1, 0, -1.9),
            landed=LandedState.IN_AIR,
        )
    )
    assert mission.phase is BroadState.PRECISION_LANDING
    dispatch(ledger, mission.state.active_operation_id, 3.0, acknowledge=True)

    mission.update(
        snapshot(
            3.1,
            ledger,
            mode=FlightMode.LAND,
            armed=True,
            position=LocalNed(1, 0, -1),
            landed=LandedState.LANDING,
        )
    )
    mission.update(snapshot(4.0, ledger, mode=FlightMode.LAND, armed=False))
    assert mission.phase is BroadState.COMPLETED

    kinds = Counter(type(envelope.command) for envelope in tuple(commands.queue))
    assert kinds == Counter(
        {
            SetMode: 2,
            Arm: 1,
            Takeoff: 1,
            MoveToLocalNed: 1,
            PrecisionLand: 1,
        }
    )
    assert len(ledger.snapshot()) == 6
