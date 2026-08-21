"""Validated runtime configuration boundary."""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from src.commands.types import FlightMode
from src.mission.protocols import MissionParameters
from src.navigation.ned import BodyFrdDisplacement
from src.navigation.search_route import SearchRoute


def load_config(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("configuration root must be a mapping")
    validate_config(config)
    return config


def validate_config(config: dict) -> None:
    required_sections = {
        "flight",
        "mission",
        "freshness_s",
        "motion",
        "timeouts",
        "comm",
        "shutdown",
        "serial",
        "ros2",
    }
    missing = required_sections - config.keys()
    if missing:
        raise ValueError(f"missing configuration sections: {sorted(missing)}")

    positive_paths = (
        ("flight", "takeoff_altitude_m"),
        ("mission", "update_rate_hz"),
        ("mission", "journal_capacity"),
        ("mission", "event_capacity"),
        ("mission", "ledger_capacity"),
        ("motion", "departure_threshold_m"),
        ("motion", "position_tolerance_m"),
        ("motion", "settle_dwell_s"),
        ("comm", "heartbeat_rate_hz"),
        ("comm", "telemetry_publish_rate_hz"),
        ("comm", "command_max_age_s"),
        ("comm", "result_correlation_s"),
        ("shutdown", "grounded_evidence_max_age_s"),
    )
    for section, key in positive_paths:
        _positive(config[section][key], f"{section}.{key}")
    for key, value in config["freshness_s"].items():
        _positive(value, f"freshness_s.{key}")
    for key, value in config["timeouts"].items():
        _positive(value, f"timeouts.{key}")
    if config["comm"]["heartbeat_rate_hz"] < 1.0:
        raise ValueError("comm.heartbeat_rate_hz must be at least 1 Hz")
    for key in ("command_capacity", "result_capacity"):
        value = config["comm"][key]
        if not isinstance(value, int) or isinstance(value, bool) or value < 16:
            raise ValueError(f"comm.{key} must be an integer >= 16")
    for path in ("journal_capacity", "event_capacity", "ledger_capacity"):
        value = config["mission"][path]
        if not isinstance(value, int) or value < 16:
            raise ValueError(f"mission.{path} must be an integer >= 16")
    ratio = config["flight"].get("takeoff_reached_ratio", 0.9)
    if (
        isinstance(ratio, bool)
        or not isinstance(ratio, (int, float))
        or not math.isfinite(float(ratio))
        or ratio <= 0
        or ratio > 1
    ):
        raise ValueError("flight.takeoff_reached_ratio must be in (0, 1]")
    mode = config["mission"].get("startup_mode", "LOITER")
    if mode != FlightMode.LOITER.value:
        raise ValueError("initial runtime currently requires LOITER startup mode")
    if config.get("landing_target_relay", {}).get("enabled", False):
        raise ValueError(
            "landing target relay remains gated until a validated adapter is implemented"
        )
    build_mission_parameters(config)


def build_mission_parameters(config: dict) -> MissionParameters:
    motion = config["motion"]
    displacements = [
        BodyFrdDisplacement(
            float(item["forward"]), float(item["right"]), float(item["down"])
        )
        for item in config["mission"]["search_route_body_frd_m"]
    ]
    route = SearchRoute(
        displacements,
        float(motion["departure_threshold_m"]),
        float(motion["position_tolerance_m"]),
    )
    timeouts = config["timeouts"]
    return MissionParameters(
        startup_mode=FlightMode(config["mission"].get("startup_mode", "LOITER")),
        takeoff_altitude_m=float(config["flight"]["takeoff_altitude_m"]),
        takeoff_reached_ratio=float(
            config["flight"].get("takeoff_reached_ratio", 0.9)
        ),
        route=route,
        departure_threshold_m=float(motion["departure_threshold_m"]),
        position_tolerance_m=float(motion["position_tolerance_m"]),
        settle_dwell_s=float(motion["settle_dwell_s"]),
        connection_timeout_s=float(timeouts["connection_s"]),
        ekf_timeout_s=float(timeouts["ekf_s"]),
        mode_change_timeout_s=float(timeouts["mode_change_s"]),
        arm_timeout_s=float(timeouts["arm_s"]),
        takeoff_timeout_s=float(timeouts["takeoff_s"]),
        waypoint_timeout_s=float(timeouts["waypoint_s"]),
        landing_timeout_s=float(timeouts["landing_s"]),
    )


def _positive(value, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    if not math.isfinite(float(value)) or value <= 0:
        raise ValueError(f"{name} must be positive and finite")
