from src.commands.types import Arm, SetMode
from src.mission.model import BroadState
from src.mission.states.preflight import PreflightStep
from src.mission.states.takeoff import TakeoffStep
from tests.mission.helpers import coordinator, snapshot


def test_one_step_or_outer_transition_per_tick_and_no_destination_update():
    mission, clock, commands, ledger = coordinator()
    ready = snapshot(0, ledger)

    mission.update(ready)
    assert mission.phase is BroadState.PREFLIGHT
    assert mission.state.step is PreflightStep.WAITING_FOR_EKF
    assert commands.qsize() == 0

    mission.update(ready)
    assert mission.phase is BroadState.TAKEOFF
    assert mission.state.step is TakeoffStep.SETTING_LOITER
    assert commands.qsize() == 1
    assert isinstance(commands.queue[0].command, SetMode)

    mission.update(snapshot(0, ledger, mode=mission.parameters.startup_mode))
    assert mission.phase is BroadState.TAKEOFF
    assert mission.state.step is TakeoffStep.ARMING
    assert commands.qsize() == 2
    assert isinstance(commands.queue[1].command, Arm)


def test_completion_evidence_at_deadline_beats_timeout():
    mission, clock, commands, ledger = coordinator()
    ready = snapshot(0, ledger)
    mission.update(ready)
    mission.update(ready)
    mission.update(snapshot(3, ledger, mode=mission.parameters.startup_mode))
    assert mission.state.step is TakeoffStep.ARMING
