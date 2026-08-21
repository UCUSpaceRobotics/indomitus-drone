"""Nonblocking MAVLink comm process and typed command dispatcher."""

from __future__ import annotations

import queue
import signal
import time
from dataclasses import dataclass

from src.comm.messages import (
    CommandEnvelope,
    CommHealth,
    CommHealthKind,
    CommResult,
    TelemetryPacket,
)
from src.commands.ledger import OperationStatus
from src.commands.types import (
    Arm,
    LandHere,
    LandingTarget,
    MoveToLocalNed,
    PrecisionLand,
    SetMode,
    Takeoff,
)


ACK_COMMAND_IDS = {
    Arm: 400,  # MAV_CMD_COMPONENT_ARM_DISARM
    Takeoff: 22,  # MAV_CMD_NAV_TAKEOFF
    PrecisionLand: 21,  # MAV_CMD_NAV_LAND
    LandHere: 21,
}


@dataclass(frozen=True)
class CommLoopConfig:
    heartbeat_rate_hz: float = 2.0
    telemetry_publish_rate_hz: float = 10.0
    command_max_age_s: float = 0.5
    result_correlation_s: float = 3.0
    dispatch_budget: int = 1

    def __post_init__(self) -> None:
        if self.heartbeat_rate_hz < 1.0:
            raise ValueError("GCS heartbeat rate must be at least 1 Hz")
        if self.telemetry_publish_rate_hz <= 0:
            raise ValueError("telemetry publish rate must be positive")
        if self.command_max_age_s <= 0 or self.result_correlation_s <= 0:
            raise ValueError("comm deadlines must be positive")
        if self.dispatch_budget < 1:
            raise ValueError("dispatch budget must be positive")


def dispatch_envelope(client, envelope: CommandEnvelope, now: float) -> CommResult:
    """Perform one low-level send attempt for one fresh typed envelope."""
    command = envelope.command
    if envelope.operation_id != command.operation_id:
        return CommResult(
            envelope.operation_id,
            OperationStatus.DROPPED,
            now,
            "envelope and command operation IDs differ",
            0,
        )
    try:
        if isinstance(command, SetMode):
            sent = client.set_mode(command.mode.value)
        elif isinstance(command, Arm):
            sent = client.arm(True)
        elif isinstance(command, Takeoff):
            sent = client.takeoff(command.altitude_m)
        elif isinstance(command, MoveToLocalNed):
            sent = client.send_local_ned_position_target(
                command.north_m, command.east_m, command.down_m
            )
        elif isinstance(command, PrecisionLand):
            sent = client.precision_land()
        elif isinstance(command, LandHere):
            sent = client.land_here()
        elif isinstance(command, LandingTarget):
            sent = client.send_landing_target(
                (command.forward_m, command.right_m, command.down_m), False
            )
        else:
            return CommResult(
                envelope.operation_id,
                OperationStatus.DROPPED,
                now,
                f"unsupported command type: {type(command).__name__}",
                0,
            )
        if sent is False:
            return CommResult(
                envelope.operation_id,
                OperationStatus.TRANSPORT_FAILED,
                now,
                "client rejected command before low-level send",
                0,
            )
    except Exception as exc:
        return CommResult(
            envelope.operation_id,
            OperationStatus.TRANSPORT_FAILED,
            now,
            str(exc),
            1,
        )
    return CommResult(
        envelope.operation_id,
        OperationStatus.DISPATCHED,
        now,
        attempted_sends=1,
    )


def stale_envelope_result(
    envelope: CommandEnvelope, now: float, max_age_s: float
) -> CommResult | None:
    if now - envelope.created_at <= max_age_s:
        return None
    return CommResult(
        envelope.operation_id,
        OperationStatus.DROPPED,
        now,
        "stale before dispatch",
        0,
    )


def service_due_heartbeat(client, now: float, last_sent: float, interval: float) -> float:
    if now - last_sent >= interval:
        client.send_gcs_heartbeat()
        return now
    return last_sent


def comm_process_loop(
    telemetry_queue,
    command_queue,
    connection_string="/dev/ttyAMA0",
    baudrate=921600,
    result_queue=None,
    health_queue=None,
    stop_event=None,
    comm_config=None,
):
    """Own UART, heartbeat, receive parsing, and bounded one-shot dispatch."""
    legacy_mode = (
        result_queue is None
        and health_queue is None
        and stop_event is None
        and comm_config is None
    )
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    config = (
        comm_config
        if isinstance(comm_config, CommLoopConfig)
        else CommLoopConfig(**(comm_config or {}))
    )
    result_queue = result_queue or _NullQueue()
    health_queue = health_queue or _NullQueue()
    stop_event = stop_event or _NeverSet()
    health_queue.put_nowait(CommHealth(CommHealthKind.STARTING, time.monotonic()))

    try:
        from src.comm.mavlink_client import PixhawkClient

        client = PixhawkClient(connection_string, baudrate)
        if not client.wait_for_heartbeat(timeout=15.0):
            health_queue.put_nowait(
                CommHealth(
                    CommHealthKind.FAILED,
                    time.monotonic(),
                    "Pixhawk heartbeat timeout",
                )
            )
            return
        client.request_data_streams(rate_hz=10)
        client.request_pose_stream(rate_hz=20)
    except Exception as exc:
        health_queue.put_nowait(
            CommHealth(CommHealthKind.FAILED, time.monotonic(), str(exc))
        )
        return

    heartbeat_interval = 1.0 / config.heartbeat_rate_hz
    telemetry_interval = 1.0 / config.telemetry_publish_rate_hz
    last_heartbeat = 0.0
    last_telemetry = 0.0
    outstanding: dict[int, tuple[str, float]] = {}
    health_queue.put_nowait(CommHealth(CommHealthKind.HEALTHY, time.monotonic()))

    try:
        while not stop_event.is_set():
            now = time.monotonic()

            telemetry = client.get_telemetry_tick()
            _process_protocol_events(
                client.drain_protocol_events(), outstanding, result_queue
            )

            last_heartbeat = service_due_heartbeat(
                client, now, last_heartbeat, heartbeat_interval
            )

            for _ in range(config.dispatch_budget):
                try:
                    envelope = command_queue.get_nowait()
                except queue.Empty:
                    break
                if not isinstance(envelope, CommandEnvelope):
                    timestamp = envelope.get("timestamp", 0.0)
                    if time.time() - timestamp > config.command_max_age_s:
                        continue
                    _dispatch_legacy(client, envelope)
                    continue
                stale = stale_envelope_result(
                    envelope, now, config.command_max_age_s
                )
                if stale is not None:
                    result_queue.put_nowait(stale)
                    continue
                ack_id = ACK_COMMAND_IDS.get(type(envelope.command))
                if ack_id is not None and ack_id in outstanding:
                    result_queue.put_nowait(
                        CommResult(
                            envelope.operation_id,
                            OperationStatus.DROPPED,
                            now,
                            "same MAV_CMD already awaiting ACK",
                            0,
                        )
                    )
                    continue
                result = dispatch_envelope(client, envelope, now)
                result_queue.put_nowait(result)
                if (
                    result.status is OperationStatus.DISPATCHED
                    and ack_id is not None
                ):
                    outstanding[ack_id] = (
                        envelope.operation_id,
                        now + config.result_correlation_s,
                    )

            for command_id, (operation_id, deadline) in tuple(outstanding.items()):
                if now >= deadline:
                    result_queue.put_nowait(
                        CommResult(
                            operation_id,
                            OperationStatus.UNKNOWN,
                            now,
                            "ACK correlation timeout",
                            1,
                        )
                    )
                    del outstanding[command_id]

            if now - last_telemetry >= telemetry_interval:
                _replace_latest(
                    telemetry_queue,
                    dict(telemetry)
                    if legacy_mode
                    else TelemetryPacket(dict(telemetry), now),
                )
                last_telemetry = now

            time.sleep(0.002)
    except Exception as exc:
        health_queue.put_nowait(
            CommHealth(CommHealthKind.FAILED, time.monotonic(), str(exc))
        )
    finally:
        health_queue.put_nowait(CommHealth(CommHealthKind.STOPPED, time.monotonic()))


def _process_protocol_events(events, outstanding, result_queue) -> None:
    for event in events:
        if event.get("type") != "command_ack":
            continue
        command_id = int(event["command"])
        pending = outstanding.get(command_id)
        if pending is None:
            continue
        if int(event["result"]) == 5:  # MAV_RESULT_IN_PROGRESS
            continue
        outstanding.pop(command_id)
        operation_id, _ = pending
        result_queue.put_nowait(
            CommResult(
                operation_id,
                OperationStatus.ACKNOWLEDGED
                if event["accepted"]
                else OperationStatus.REJECTED,
                float(event["received_at"]),
                f"MAV_RESULT={event['result']}",
                1,
            )
        )


def _replace_latest(target_queue, value) -> None:
    while True:
        try:
            target_queue.get_nowait()
        except queue.Empty:
            break
    try:
        target_queue.put_nowait(value)
    except queue.Full:
        pass


class _NullQueue:
    def put_nowait(self, value) -> None:
        pass


class _NeverSet:
    def is_set(self) -> bool:
        return False


# Legacy motor-capable bench scripts retain their existing raw API. Production
# runtime uses only CommandEnvelope and does not call these helpers.
def dispatch_command(client, cmd):
    return _dispatch_legacy(client, cmd)


def _dispatch_legacy(client, cmd):
    action = cmd.get("action")
    if action == "arm":
        return client.arm(state=cmd.get("state", True))
    if action == "set_mode":
        return client.set_mode(mode_name=cmd.get("mode", "GUIDED"))
    if action == "takeoff":
        return client.takeoff(altitude_m=cmd.get("altitude", 2.0))
    if action == "land":
        return client.precision_land()
    if action == "move_local_pos":
        return client.send_position_target_local_ned(
            dx_m=cmd.get("dx", 0.0),
            dy_m=cmd.get("dy", 0.0),
            dz_m=cmd.get("dz", 0.0),
        )
    if action == "set_local_position":
        return client.send_local_ned_position_target(
            x_m=cmd.get("x", 0.0),
            y_m=cmd.get("y", 0.0),
            z_m=cmd.get("z", 0.0),
        )
    if action == "move_local_vel":
        return client.send_velocity_target_body_ned(
            vx_m_s=cmd.get("vx", 0.0),
            vy_m_s=cmd.get("vy", 0.0),
            vz_m_s=cmd.get("vz", 0.0),
        )
    if action in {"land_on_target", "send_landing_target"}:
        target = cmd.get("target")
        if target is None:
            raise ValueError(f"{action} command has no target")
        return client.send_landing_target(
            tuple(target), cmd.get("initiate_landing", action == "land_on_target")
        )
    raise ValueError(f"unknown command action: {action}")


def create_command(action, **kwargs):
    command = {"action": action, "timestamp": time.time()}
    command.update(kwargs)
    return command
