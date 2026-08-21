from src.navigation.motion_tracker import MotionStage, MotionState, update_motion
from src.navigation.ned import LocalNed


def update(state, now, position, fresh=True, dispatched=True):
    return update_motion(
        state,
        now=now,
        position=position,
        position_fresh=fresh,
        dispatched=dispatched,
        dispatch_failed=False,
        departure_threshold_m=0.15,
        position_tolerance_m=0.2,
        settle_dwell_s=1.0,
    )


def test_departure_required_and_wiggle_resets_dwell():
    start = LocalNed(0, 0, 0)
    endpoint = LocalNed(1, 0, 0)
    state = MotionState(start, endpoint, 10)
    state = update(state, 0, endpoint)
    assert state.stage is MotionStage.SETTLING_AT_TARGET
    assert state.departed_at == 0
    state = update(state, 0.5, LocalNed(0.7, 0, 0))
    assert state.stage is MotionStage.IN_TRANSIT
    assert state.dwell_started_at is None
    state = update(state, 1.0, endpoint)
    state = update(state, 2.0, endpoint)
    assert state.stage is MotionStage.REACHED


def test_start_inside_target_tolerance_cannot_shortcut_departure():
    state = MotionState(LocalNed(0, 0, 0), LocalNed(0.18, 0, 0), 2)
    state = update(state, 0, LocalNed(0.05, 0, 0))
    assert state.stage is MotionStage.WAITING_FOR_DEPARTURE


def test_completion_at_deadline_beats_timeout_and_stale_pose_cannot_complete():
    state = MotionState(LocalNed(0, 0, 0), LocalNed(1, 0, 0), 2)
    state = update(state, 0, LocalNed(0.5, 0, 0))
    state = update(state, 1, LocalNed(1, 0, 0))
    state = update(state, 2, LocalNed(1, 0, 0))
    assert state.stage is MotionStage.REACHED

    stale = MotionState(LocalNed(0, 0, 0), LocalNed(1, 0, 0), 2)
    stale = update(stale, 2, LocalNed(1, 0, 0), fresh=False)
    assert stale.stage is MotionStage.FAILED
    assert stale.timed_out


def test_stale_interval_resets_continuous_dwell():
    state = MotionState(LocalNed(0, 0, 0), LocalNed(1, 0, 0), 10)
    state = update(state, 0, LocalNed(0.5, 0, 0))
    state = update(state, 1, LocalNed(1, 0, 0))
    assert state.stage is MotionStage.SETTLING_AT_TARGET
    state = update(state, 2, None, fresh=False)
    assert state.stage is MotionStage.IN_TRANSIT
    assert state.dwell_started_at is None
    state = update(state, 2.1, LocalNed(1, 0, 0))
    assert state.stage is MotionStage.SETTLING_AT_TARGET
    assert state.dwell_started_at == 2.1
