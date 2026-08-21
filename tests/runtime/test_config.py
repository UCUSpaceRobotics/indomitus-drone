from copy import deepcopy

import pytest

from src.runtime.config import load_config, validate_config


def test_repository_config_is_valid_and_relay_is_gated():
    config = load_config("config/mission_params.yaml")
    assert config["landing_target_relay"]["enabled"] is False
    assert config["mission"]["num_landings"] == 3  # ignored rollback profile


def test_invalid_heartbeat_and_route_are_rejected():
    config = load_config("config/mission_params.yaml")
    invalid = deepcopy(config)
    invalid["comm"]["heartbeat_rate_hz"] = 0.5
    with pytest.raises(ValueError):
        validate_config(invalid)

    invalid = deepcopy(config)
    invalid["mission"]["search_route_body_frd_m"][0]["forward"] = 0.15
    with pytest.raises(ValueError):
        validate_config(invalid)


def test_relay_cannot_enable_without_validated_depth_source():
    config = load_config("config/mission_params.yaml")
    config["landing_target_relay"]["enabled"] = True
    with pytest.raises(ValueError):
        validate_config(config)


@pytest.mark.parametrize(
    ("section", "key", "value"),
    [
        ("comm", "command_capacity", 0),
        ("comm", "result_capacity", -1),
        ("flight", "takeoff_reached_ratio", float("nan")),
    ],
)
def test_unsafe_capacity_and_nonfinite_ratio_are_rejected(section, key, value):
    config = load_config("config/mission_params.yaml")
    config[section][key] = value
    with pytest.raises(ValueError):
        validate_config(config)
