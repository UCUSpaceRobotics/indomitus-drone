import queue

from src.activities.manager import ActivityManager
from src.commands.gateway import CommandGateway
from src.commands.ledger import OperationLedger
from src.commands.types import FlightMode
from src.mission.coordinator import MissionCoordinator
from src.mission.model import MissionSnapshot
from src.mission.protocols import MissionParameters
from src.navigation.ned import BodyFrdDisplacement, LocalNed
from src.navigation.search_route import SearchRoute
from src.observations.model import LandedState
from src.observations.store import ObservationStore
from tests.fakes.clock import FakeClock


def parameters(route_legs=1):
    route = SearchRoute(
        [BodyFrdDisplacement(1, 0, 0) for _ in range(route_legs)], 0.15, 0.2
    )
    return MissionParameters(
        startup_mode=FlightMode.LOITER,
        takeoff_altitude_m=2.0,
        takeoff_reached_ratio=0.9,
        route=route,
        departure_threshold_m=0.15,
        position_tolerance_m=0.2,
        settle_dwell_s=1.0,
        connection_timeout_s=5.0,
        ekf_timeout_s=5.0,
        mode_change_timeout_s=3.0,
        arm_timeout_s=3.0,
        takeoff_timeout_s=5.0,
        waypoint_timeout_s=5.0,
        landing_timeout_s=5.0,
    )


def snapshot(
    now,
    ledger,
    *,
    mode=FlightMode.LOITER,
    armed=False,
    position=LocalNed(0, 0, 0),
    landed=LandedState.ON_GROUND,
    control=False,
    comm=True,
    fresh=True,
):
    return MissionSnapshot(
        now=now,
        comm_healthy=comm,
        heartbeat_fresh=fresh,
        mode=mode if fresh else None,
        armed=armed if fresh else None,
        ekf_fresh=fresh,
        ekf_healthy=True if fresh else None,
        position_fresh=fresh,
        position=position if fresh else None,
        yaw_fresh=fresh,
        yaw_rad=0.0 if fresh else None,
        landed_fresh=fresh,
        landed_state=landed if fresh else None,
        operations=ledger.snapshot(),
        control_requested=control,
    )


def coordinator(route_legs=1):
    clock = FakeClock()
    command_queue = queue.Queue()
    ledger = OperationLedger(64)
    gateway = CommandGateway(command_queue, ledger, clock)
    activities = ActivityManager(None, production_relay_enabled=False)
    store = ObservationStore()
    initial = snapshot(0, ledger, fresh=False, comm=False)
    result = MissionCoordinator(
        "test",
        parameters(route_legs),
        gateway,
        activities,
        store,
        64,
        initial,
    )
    return result, clock, command_queue, ledger
