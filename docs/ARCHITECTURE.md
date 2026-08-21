# Architecture — Indomitus Drone

## Status

`main.py` uses lifecycle runtime described here. Deterministic unit and fake-runtime tests exist; ArduPilot SITL, Raspberry Pi/Pixhawk props-removed, and controlled-flight rollout gates remain mandatory before flight use.

Precision `LANDING_TARGET` relay remains deliberately disabled. Current `/erc/vision_targets` `geometry_msgs/msg/Point.z` carries marker ID, not target depth. Runtime never invents BODY_FRD Z from marker ID or local altitude. Search exhaustion therefore issues opportunistic precision LAND, which ArduCopter may execute as normal landing when no validated target stream exists.

Legacy `src/navigation/state_machine.py` and motor-capable root scripts remain for preservation/rollback. Production `main.py` does not import legacy controller. Legacy removal requires explicit reconciliation of existing unstaged work.

## Component ownership

```text
ROS vision ─┐
            ├─> RuntimeSupervisor ─> ObservationStore + EventInbox
Comm child ─┘            │
                         ├─> ActivityManager ─> LandingTargetRelay (gated)
                         └─> MissionCoordinator ─> active state object
                                                   │ entry only
                                                   v
                                            CommandGateway
                                                   │ typed IPC
                                                   v
                                              Comm child
                                                   │ MAVLink
                                                   v
                                                Pixhawk
```

- `RuntimeSupervisor`: parent SIGINT ownership, input ordering, comm/process health, runtime lifetime, diagnostics, and fresh grounded/disarmed shutdown gate.
- `MissionCoordinator`: broad lifecycle, transition/failure policy, mode preemption, step entry, bounded journal, and one-advancement-per-update budget.
- State objects: effect-free `update()` evaluation. Effects occur only in coordinator-invoked state/step entry.
- `CommandGateway`: semantic operation uniqueness, record-before-submit, one IPC submission attempt, cancellation metadata.
- Comm child: nonblocking MAVLink receive, independent 2 Hz GCS heartbeat, bounded command dispatch, ACK correlation, and telemetry/result/health publication.
- `ObservationStore`: latest timestamped values and caller-owned freshness checks.
- `EventInbox`: bounded, owner-addressed consumable events with visible overflow.
- `ActivityManager`: continuous activity scope. Relay scope is Search plus PrecisionLanding when production gate is enabled.
- Pixhawk/ArduCopter: stabilization, mode execution, waypoint guidance, precision alignment/descent, landed-state reporting, and failsafes.

Domain packages do not import ROS, pymavlink, multiprocessing, or wall clocks.

## Lifecycle

```text
Preflight -> Takeoff -> Search -> PrecisionLanding -> Completed
                  \         \              \
                   \         -> LandHere     -> AirborneFault
                    -> LandHere -> Faulted          |
                                                   -> Faulted when grounded

Active state + Ctrl+C/unexpected fresh mode -> Yielded
```

Broad states:

- `Preflight`: wait for fresh heartbeat/healthy comm, configured LOITER mode, healthy EKF, local position, and yaw.
- `Takeoff`: `SETTING_LOITER -> ARMING -> SETTING_GUIDED -> ASCENDING`.
- `Search`: each fixed relative leg uses `WAITING_FOR_FRESH_POSE -> MOVING_LEG`.
- `PrecisionLanding`: `LAND_COMMAND_PENDING -> DESCENDING`, with one opportunistic LAND operation.
- `LandHere`: controlled pre-precision failure path with one non-precision LAND operation.
- `Completed`: successful, fresh on-ground and disarmed terminal.
- `Faulted`: unsuccessful grounded terminal.
- `Yielded`: passive mission terminal after operator/control preemption; no LAND and no resume.
- `AirborneFault`: passive observation-only fault; no effects, no second LAND, transitions to `Faulted` only on fresh on-ground and disarmed evidence.

One update applies at most one internal step advancement or one outer transition. Destination state entry may submit its one operation, but destination `update()` waits until next tick. After preemption checks, completion evidence is evaluated before timeout on same tick.

## Mode preemption

Fresh unexpected mode telemetry preempts active mission work. Source and destination modes are both authorized while a commanded mode transition is pending. RC takeover and failsafe mode changes are intentionally treated alike because HEARTBEAT does not identify origin. Same-mode takeover remains undetectable without dedicated RC input.

Ctrl+C is converted to a parent-owned control event. It cancels active operation metadata, stops activities, enters `Yielded`, and issues no LAND. Comm child ignores parent SIGINT so heartbeat and telemetry continue while transport remains usable.

## One-shot command contract

Typed commands:

- `SetMode(LOITER|GUIDED)`
- `Arm`
- `Takeoff`
- `MoveToLocalNed`
- `PrecisionLand` (`MAV_CMD_NAV_LAND` param2 `1`)
- `LandHere` (`MAV_CMD_NAV_LAND` param2 `0`)
- `LandingTarget` (`MAV_FRAME_BODY_FRD`, activity effect)

Every deterministic operation ID is recorded before one `put_nowait()` IPC submission attempt. Duplicate IDs fail before queueing and remain forbidden after bounded ledger record eviction. Comm performs zero low-level sends for stale/invalid/pre-dispatch failures, or one send attempt after dispatch begins. No retry or retransmission exists.

Operation status is distinct from physical completion:

```text
recorded -> queued -> dispatched -> acknowledged/rejected/unknown
                    \-> dropped/transport-failed
```

- `MoveToLocalNed`, `LandingTarget`, and `SetMode`: successful `dispatched` is terminal transport evidence.
- `Arm`, `Takeoff`, `PrecisionLand`, and `LandHere`: await correlated ACK/rejection or correlation timeout.
- Mode telemetry, armed state, waypoint motion, and touchdown remain separate physical evidence.
- Cancellation cannot unsend already queued/in-flight transport. Late results update history but never reactivate `Yielded`.

## Search route and motion evidence

At each leg start, fresh local NED pose `(N0, E0, D0)` and yaw `ψ` are captured once. Configured body-forward/right/down displacement `(f, r, d)` resolves to:

```text
delta_north = f*cos(ψ) - r*sin(ψ)
delta_east  = f*sin(ψ) + r*cos(ψ)
endpoint    = (N0 + delta_north, E0 + delta_east, D0 + d)
```

Endpoint is frozen and submitted once as absolute `MAV_FRAME_LOCAL_NED`. Route config rejects nonfinite, zero, too-short, or arrival/departure-ambiguous legs.

Motion stages:

```text
WAITING_FOR_DISPATCH -> WAITING_FOR_DEPARTURE -> IN_TRANSIT
  -> SETTLING_AT_TARGET -> REACHED
```

Departure from captured start is mandatory. Fresh position must remain continuously in target tolerance for configured dwell. Leaving tolerance resets dwell. Velocity alone never succeeds. Public timeout remains `WAYPOINT_TIMEOUT`. Search succeeds only after every configured leg reaches; marker observations never end or replan Search.

## Vision and target relay gate

Wire contract on `/erc/vision_targets`:

- `z=101/102`: marker ID; `x/y` carry camera-frame offsets.
- `z<0`: probe event with world `x/y`.
- `z=0`: no detection.

`VisionBridge` assigns every message a monotonic receive time and unique process-local identity. Existing mounting correction negates camera X once. Relay mapping is camera Y to BODY_FRD forward X and corrected camera X to BODY_FRD right Y.

Relay accepts marker 102 only when an independently validated, fresh, finite, positive BODY_FRD down-distance exists. Missing/stale/nonfinite/nonpositive Z creates no operation. Production config keeps relay disabled until such adapter and physical sign/freshness evidence exist.

## MAVLink process

Comm child:

1. Ignores parent SIGINT.
2. Connects and parses initial HEARTBEAT mode/armed state.
3. Requests pose, EKF, RC, status, and `EXTENDED_SYS_STATE` streams.
4. Processes at most 100 received messages per receive tick.
5. Services GCS heartbeat before bounded command dispatch; configured rate is 2 Hz and validator rejects rates below 1 Hz.
6. Dispatches at most one command envelope per loop by default.
7. Correlates one unresolved ACK operation per MAV command ID; `IN_PROGRESS` remains nonterminal.
8. Publishes latest telemetry plus operation and health results.

Fresh `EXTENDED_SYS_STATE=ON_GROUND` plus fresh disarmed HEARTBEAT gates mission completion and normal process shutdown. Altitude alone is never touchdown evidence.

## Runtime lifetime

Mission terminality and process shutdown are separate. Runtime remains alive after `Completed`, `Faulted`, `Yielded`, or `AirborneFault` until fresh on-ground and disarmed evidence opens safe gate. Known-dead comm is visible and does not create impossible LAND operations or replay old operations. Initial policy provides no automatic recovery. Explicit programmatic `force_shutdown(actor, reason)` records audit data before bypassing safe gate.

## Configuration

`config/mission_params.yaml` validates:

- positive finite rates, freshness limits, tolerances, dwell, deadlines, and altitude;
- heartbeat rate at least 1 Hz;
- integer bounded queue/journal/event/ledger capacities;
- fixed nonempty route geometry against departure and arrival thresholds;
- LOITER startup mode;
- disabled relay while no validated down-distance adapter exists.

Legacy rollback keys remain under existing names but are ignored by new coordinator.

## Repository map

- `main.py`: thin production entrypoint.
- `src/runtime/`: composition, validated config, monotonic clock, supervisor.
- `src/mission/`: coordinator, results, models, transitions, journal, state objects.
- `src/commands/`: typed commands, ledger, gateway.
- `src/observations/`: models, latest-value store, event inbox.
- `src/activities/`: activity manager and gated landing-target relay.
- `src/navigation/`: NED transform, fixed route, pure motion tracker; legacy controller retained separately.
- `src/comm/`: typed IPC, nonblocking comm process, MAVLink adapter.
- `src/ros_bridge/`: identified vision observations and legacy probe accumulation.
- `src/diagnostics/`: status projection.
- `tests/{commands,observations,navigation,activities,mission,comm,integration,runtime}/`: deterministic unit/fake-runtime coverage.

## Verification and rollout

Safe local verification:

```bash
python3 -m pytest tests/activities tests/comm tests/commands tests/integration tests/mission tests/navigation tests/observations tests/runtime -q
python3 -m unittest tests.test_square_controller -v
python3 -m unittest tests.test_led_indicator -v
python3 -m pytest tests/test_grid_mapper.py -q
```

Do not use `main.py`, `test_square_flight.py`, or root landing-target scripts as routine verification; they can command real motors.

Remaining flight gates:

1. ArduPilot SITL: ACK behavior, absolute setpoint persistence, mode preemption, LAND param2, landed-state transitions, comm interruption.
2. Props removed: UART timing, heartbeat under load, child SIGINT isolation, camera/body signs, yaw/NED transform, shutdown path.
3. Controlled flight: route bounds, departure/dwell, opportunistic landing, target relay after depth gate, and LandHere failure policy.
4. Owner-approved legacy reconciliation and stale-document cleanup.

`lavish/state-machine-architecture.html` and `lavish/state-machine-implementation-plan.md` contain design rationale and full rollout agreement. This file describes implemented source plus explicit unverified gates.
