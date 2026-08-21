#!/usr/bin/env python3
"""Indomitus mission runtime entrypoint.

This executable can command real motors through /dev/ttyAMA0. Use
scripts/start_mission.sh on configured flight hardware only.
"""

from __future__ import annotations

from pathlib import Path

from src.runtime.composition import create_runtime
from src.runtime.config import load_config as load_validated_config


REPOSITORY_ROOT = Path(__file__).resolve().parent


def load_config(path: str = "config/mission_params.yaml") -> dict:
    config_path = REPOSITORY_ROOT / path
    config = load_validated_config(config_path)
    print(f"[MAIN] Configuration loaded from {config_path}")
    return config


def main() -> None:
    config = load_config()
    runtime = create_runtime(config)
    final_status = runtime.run()
    print(f"[MAIN] Mission runtime stopped in {final_status.phase.value}")


if __name__ == "__main__":
    main()
