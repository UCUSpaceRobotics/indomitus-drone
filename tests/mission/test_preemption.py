from src.commands.ledger import OperationStatus
from src.commands.types import FlightMode
from src.mission.model import BroadState
from src.mission.states.terminal import PassiveState
from tests.mission.helpers import coordinator, snapshot


def enter_takeoff(mission, ledger):
    ready = snapshot(0, ledger)
    mission.update(ready)
    mission.update(ready)


def test_unexpected_mode_and_control_preempt_before_state_work():
    mission, clock, commands, ledger = coordinator()
    enter_takeoff(mission, ledger)
    active_id = mission.state.active_operation_id

    status = mission.update(snapshot(0, ledger, mode=FlightMode.STABILIZE))

    assert status.phase is BroadState.YIELDED
    assert ledger.get(active_id).cancelled
    assert commands.qsize() == 1

    mission2, _, commands2, ledger2 = coordinator()
    enter_takeoff(mission2, ledger2)
    status = mission2.update(snapshot(0, ledger2, control=True))
    assert status.phase is BroadState.YIELDED
    assert commands2.qsize() == 1


def test_late_result_cannot_reactivate_yielded():
    mission, _, _, ledger = coordinator()
    enter_takeoff(mission, ledger)
    operation_id = mission.state.active_operation_id
    mission.update(snapshot(0, ledger, control=True))
    ledger.transition(operation_id, OperationStatus.DISPATCHED, 1)
    mission.update(snapshot(1, ledger, mode=FlightMode.LOITER))
    assert mission.phase is BroadState.YIELDED


def test_airborne_fault_ignores_mode_changes_and_only_faults_when_grounded():
    mission, _, _, ledger = coordinator()
    airborne = snapshot(
        0,
        ledger,
        mode=FlightMode.GUIDED,
        armed=True,
        landed=None,
        fresh=False,
    )
    mission._transition(BroadState.AIRBORNE_FAULT, airborne, "test")
    mission.update(snapshot(1, ledger, mode=FlightMode.STABILIZE, armed=True, landed=None))
    assert mission.phase is BroadState.AIRBORNE_FAULT
    mission.update(snapshot(2, ledger, mode=FlightMode.STABILIZE))
    assert mission.phase is BroadState.FAULTED
