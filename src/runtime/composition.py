"""Production process, adapter, and lifecycle composition."""

from __future__ import annotations

import multiprocessing
import uuid

from src.activities.landing_target_relay import LandingTargetRelay
from src.activities.manager import ActivityManager
from src.commands.gateway import CommandGateway
from src.commands.ledger import OperationLedger
from src.comm.mavlink_node import CommLoopConfig, comm_process_loop
from src.mission.coordinator import MissionCoordinator
from src.mission.model import MissionSnapshot
from src.observations.event_inbox import EventInbox
from src.observations.store import ObservationStore
from src.runtime.clock import MonotonicClock
from src.runtime.config import build_mission_parameters
from src.runtime.supervisor import RuntimeSupervisor


def create_runtime(config: dict) -> RuntimeSupervisor:
    clock = MonotonicClock()
    context = multiprocessing.get_context("spawn")
    command_queue = context.Queue(maxsize=int(config["comm"]["command_capacity"]))
    telemetry_queue = context.Queue(maxsize=1)
    result_queue = context.Queue(maxsize=int(config["comm"]["result_capacity"]))
    health_queue = context.Queue(maxsize=32)
    stop_event = context.Event()
    comm_config = CommLoopConfig(
        heartbeat_rate_hz=float(config["comm"]["heartbeat_rate_hz"]),
        telemetry_publish_rate_hz=float(
            config["comm"]["telemetry_publish_rate_hz"]
        ),
        command_max_age_s=float(config["comm"]["command_max_age_s"]),
        result_correlation_s=float(config["comm"]["result_correlation_s"]),
    )
    comm_process = context.Process(
        target=comm_process_loop,
        args=(
            telemetry_queue,
            command_queue,
            config["serial"]["port"],
            int(config["serial"]["baudrate"]),
            result_queue,
            health_queue,
            stop_event,
            comm_config,
        ),
        daemon=False,
    )
    rclpy = None
    try:
        comm_process.start()
        import rclpy

        rclpy.init()
        from src.ros_bridge.vision_subscriber import VisionBridge
        from src.utils.led_indicator import LEDController

        vision = VisionBridge(
            topic=config["ros2"]["vision_topic"], grid_config=config.get("grid")
        )
        led = LEDController(config.get("led", {}))

        mission_id = uuid.uuid4().hex
        store = ObservationStore()
        inbox = EventInbox(int(config["mission"]["event_capacity"]))
        ledger = OperationLedger(int(config["mission"]["ledger_capacity"]))
        gateway = CommandGateway(command_queue, ledger, clock)
        relay = LandingTargetRelay(
            gateway,
            mission_id,
            camera_freshness_s=float(config["freshness_s"]["camera"]),
            distance_freshness_s=float(
                config["freshness_s"]["target_down_distance"]
            ),
        )
        activities = ActivityManager(
            relay,
            production_relay_enabled=bool(
                config.get("landing_target_relay", {}).get("enabled", False)
            ),
        )
        now = clock.now()
        initial = MissionSnapshot(
            now=now,
            comm_healthy=False,
            heartbeat_fresh=False,
            mode=None,
            armed=None,
            ekf_fresh=False,
            ekf_healthy=None,
            position_fresh=False,
            position=None,
            yaw_fresh=False,
            yaw_rad=None,
            landed_fresh=False,
            landed_state=None,
            operations={},
        )
        coordinator = MissionCoordinator(
            mission_id,
            build_mission_parameters(config),
            gateway,
            activities,
            store,
            int(config["mission"]["journal_capacity"]),
            initial,
        )
        return RuntimeSupervisor(
            clock=clock,
            coordinator=coordinator,
            store=store,
            inbox=inbox,
            telemetry_queue=telemetry_queue,
            result_queue=result_queue,
            health_queue=health_queue,
            comm_process=comm_process,
            stop_event=stop_event,
            vision=vision,
            led=led,
            config=config,
            rclpy_module=rclpy,
        )
    except BaseException:
        stop_event.set()
        if comm_process.pid is not None:
            comm_process.join(timeout=2.0)
            if comm_process.is_alive():
                comm_process.terminate()
                comm_process.join(timeout=1.0)
        if rclpy is not None:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        raise
