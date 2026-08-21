import queue

from src.commands.gateway import CommandGateway
from src.commands.ledger import OperationLedger, OperationStatus
from src.mission.protocols import EntryContext
from src.mission.results import AdvanceStep, Running, StateSucceeded
from src.mission.states.search import SearchState, SearchStep
from src.navigation.ned import LocalNed
from tests.fakes.clock import FakeClock
from tests.mission.helpers import parameters, snapshot


def test_search_submits_each_leg_once_and_marker_cannot_complete_it():
    clock = FakeClock()
    commands = queue.Queue()
    ledger = OperationLedger(64)
    gateway = CommandGateway(commands, ledger, clock)
    state = SearchState(parameters(route_legs=1))
    initial = snapshot(0, ledger, mode=state.parameters.startup_mode)
    ctx = EntryContext("m", 0, 5, gateway, initial)
    state.enter(ctx)
    state.enter_step(state.step, ctx)

    result = state.update(initial)
    assert isinstance(result, AdvanceStep)
    state.enter_step(result.next_step, ctx)
    assert commands.qsize() == 1
    operation_id = state.active_operation_id
    ledger.transition(operation_id, OperationStatus.DISPATCHED, 0)

    # Hundreds of evaluation ticks and external marker events cannot resubmit.
    for index in range(100):
        result = state.update(
            snapshot(index / 1000, ledger, position=LocalNed(0, 0, 0))
        )
        assert isinstance(result, Running)
    assert commands.qsize() == 1

    state.update(snapshot(1, ledger, position=LocalNed(1, 0, 0)))
    result = state.update(snapshot(2, ledger, position=LocalNed(1, 0, 0)))
    assert isinstance(result, StateSucceeded)


def test_all_route_legs_are_required_for_search_success():
    clock = FakeClock()
    commands = queue.Queue()
    ledger = OperationLedger(64)
    gateway = CommandGateway(commands, ledger, clock)
    state = SearchState(parameters(route_legs=2))
    initial = snapshot(0, ledger)
    ctx = EntryContext("m", 0, 5, gateway, initial)
    state.enter(ctx)
    state.enter_step(state.step, ctx)
    result = state.update(initial)
    state.enter_step(result.next_step, ctx)
    ledger.transition(state.active_operation_id, OperationStatus.DISPATCHED, 0)
    state.update(snapshot(1, ledger, position=LocalNed(1, 0, 0)))
    result = state.update(snapshot(2, ledger, position=LocalNed(1, 0, 0)))
    assert isinstance(result, AdvanceStep)
    assert result.next_step is SearchStep.WAITING_FOR_FRESH_POSE
