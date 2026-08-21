# Indomitus Drone — ERC 2026

Autonomous quadcopter mission runtime for Raspberry Pi 5, ROS 2 Jazzy, pymavlink, Pixhawk 6C, and ArduCopter.

## Runtime architecture

```text
Simulink ROS vision -> RuntimeSupervisor -> MissionCoordinator/state objects
Pixhawk telemetry  -> ObservationStore  -> CommandGateway -> comm child -> Pixhawk
```

Lifecycle:

```text
Preflight -> Takeoff -> fixed-route Search -> PrecisionLanding -> Completed
```

Controlled failures use one `LandHere`; landing failures become passive `AirborneFault`. Ctrl+C or unexpected fresh mode yields control without issuing LAND. Typed semantic operations receive one IPC submission attempt and at most one low-level MAVLink send attempt.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for current implementation, safety invariants, and rollout gates.

## Important current gate

`LANDING_TARGET` relay is disabled until a validated BODY_FRD down-distance source exists. `/erc/vision_targets` uses `geometry_msgs/msg/Point.z` as marker ID, not depth. Current source must pass SITL and props-removed validation before flight rollout.

## Raspberry Pi launch

`main.py` can command real motors through `/dev/ttyAMA0`. Launch only on configured hardware:

```bash
./scripts/start_mission.sh
```

Launcher sources ROS 2 workspaces, sets `ROS_DOMAIN_ID=42`, activates system-site-packages virtual environment, and preserves environment through `sudo -E`.

## Safe focused verification

```bash
python3 -m pytest tests/activities tests/comm tests/commands tests/integration tests/mission tests/navigation tests/observations tests/runtime -q
python3 -m unittest tests.test_square_controller -v
python3 -m unittest tests.test_led_indicator -v
python3 -m pytest tests/test_grid_mapper.py -q
```

Never run `main.py`, `test_square_flight.py`, or root landing-target scripts as routine tests.

## Hardware

- F450 airframe, DJI 2212 motors
- Pixhawk 6C running ArduCopter
- Raspberry Pi 5
- Downward Arducam IMX708
- Microair MTF-01 optical flow/range sensor
- Holybro M10 GPS/compass

## License

MIT — see [LICENSE](LICENSE).
