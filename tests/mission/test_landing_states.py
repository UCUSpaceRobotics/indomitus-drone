from src.commands.ledger import OperationStatus
from src.commands.types import LandHere, PrecisionLand, FlightMode
from src.mission.model import BroadState
from tests.mission.helpers import coordinator, snapshot


def test_precision_landing_timeout_enters_airborne_fault_without_second_land():
    mission, clock, commands, ledger = coordinator()
    airborne = snapshot(
        0,
        ledger,
        mode=FlightMode.GUIDED,
        armed=True,
        landed=None,
        fresh=True,
    )
    mission._transition(BroadState.PRECISION_LANDING, airborne, "route-exhausted")
    assert commands.qsize() == 1
    assert isinstance(commands.queue[0].command, PrecisionLand)

    mission.update(
        snapshot(
            5,
            ledger,
            mode=FlightMode.GUIDED,
            armed=True,
            landed=None,
            fresh=True,
        )
    )
    assert mission.phase is BroadState.AIRBORNE_FAULT
    assert commands.qsize() == 1


def test_search_failure_enters_land_here_once_then_failure_never_retries():
    mission, clock, commands, ledger = coordinator()
    airborne = snapshot(
        0,
        ledger,
        mode=FlightMode.GUIDED,
        armed=True,
        landed=None,
        fresh=True,
    )
    mission._transition(BroadState.SEARCH, airborne, "test")
    mission.update(
        snapshot(
            5,
            ledger,
            mode=FlightMode.GUIDED,
            armed=True,
            landed=None,
            fresh=False,
        )
    )
    assert mission.phase is BroadState.LAND_HERE
    assert commands.qsize() == 1
    assert isinstance(commands.queue[0].command, LandHere)

    operation_id = mission.state.active_operation_id
    ledger.transition(operation_id, OperationStatus.DISPATCHED, 6)
    ledger.transition(operation_id, OperationStatus.REJECTED, 6)
    mission.update(
        snapshot(
            6,
            ledger,
            mode=FlightMode.GUIDED,
            armed=True,
            landed=None,
            fresh=True,
        )
    )
    assert mission.phase is BroadState.AIRBORNE_FAULT
    assert commands.qsize() == 1


def test_touchdown_requires_fresh_on_ground_and_disarmed():
    mission, _, commands, ledger = coordinator()
    airborne = snapshot(
        0, ledger, mode=FlightMode.GUIDED, armed=True, landed=None, fresh=True
    )
    mission._transition(BroadState.PRECISION_LANDING, airborne, "test")
    mission.update(snapshot(1, ledger, mode=FlightMode.LAND, armed=False, fresh=False))
    assert mission.phase is BroadState.PRECISION_LANDING
    mission.update(snapshot(2, ledger, mode=FlightMode.LAND, armed=False))
    assert mission.phase is BroadState.COMPLETED


def test_landing_pending_defers_mode_preemption_without_captured_source():
    mission, _, commands, ledger = coordinator()
    stale = snapshot(0, ledger, fresh=False, comm=True)
    mission._transition(BroadState.LAND_HERE, stale, "stale-ground-evidence")
    mission.update(
        snapshot(
            1,
            ledger,
            mode=FlightMode.GUIDED,
            armed=True,
            landed=None,
            fresh=True,
        )
    )
    assert mission.phase is BroadState.LAND_HERE
    mission.update(
        snapshot(
            2,
            ledger,
            mode=FlightMode.STABILIZE,
            armed=True,
            landed=None,
            fresh=True,
        )
    )
    assert mission.phase is BroadState.YIELDED
