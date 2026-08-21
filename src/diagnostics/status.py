"""Compact human-readable mission status formatting."""

from src.mission.model import MissionStatus


def format_status(status: MissionStatus, *, comm_healthy: bool) -> str:
    step = f"/{status.step}" if status.step else ""
    operation = status.active_operation_id or "none"
    return (
        f"phase={status.phase.value}{step} waiting={status.waiting_reason or '-'} "
        f"operation={operation} comm={'healthy' if comm_healthy else 'unavailable'}"
    )
