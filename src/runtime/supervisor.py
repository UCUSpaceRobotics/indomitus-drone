"""Parent-owned input ordering, lifecycle updates, and safe shutdown gate."""

from __future__ import annotations

import queue
import signal
import time
from dataclasses import dataclass

from src.commands.ledger import OperationStatus
from src.commands.types import FlightMode
from src.comm.messages import CommHealth, CommHealthKind, CommResult, TelemetryPacket
from src.diagnostics.status import format_status
from src.mission.coordinator import MissionCoordinator
from src.mission.model import BroadState, MissionSnapshot, MissionStatus
from src.navigation.ned import LocalNed
from src.observations.model import (
    Event,
    EventType,
    LandedState,
    ObservationKey,
)
from src.observations.event_inbox import EventInbox
from src.observations.store import ObservationStore
from src.runtime.clock import Clock


@dataclass(frozen=True)
class SupervisorTick:
    status: MissionStatus
    safe_to_shutdown: bool
    force_shutdown: bool


class RuntimeSupervisor:
    def __init__(
        self,
        *,
        clock: Clock,
        coordinator: MissionCoordinator,
        store: ObservationStore,
        inbox: EventInbox,
        telemetry_queue,
        result_queue,
        health_queue,
        comm_process,
        stop_event,
        vision,
        led,
        config: dict,
        rclpy_module,
    ):
        self.clock = clock
        self.coordinator = coordinator
        self.store = store
        self.inbox = inbox
        self.telemetry_queue = telemetry_queue
        self.result_queue = result_queue
        self.health_queue = health_queue
        self.comm_process = comm_process
        self.stop_event = stop_event
        self.vision = vision
        self.led = led
        self.config = config
        self.rclpy = rclpy_module
        self.comm_healthy = False
        self._control_generation = 0
        self._processed_control_generation = 0
        self._force_shutdown = False
        self._force_actor: str | None = None
        self._force_reason: str | None = None
        self._last_status_log = 0.0

    def install_signal_handlers(self) -> None:
        signal.signal(signal.SIGINT, self._handle_sigint)

    def request_control_yield(self) -> None:
        self._control_generation += 1

    def force_shutdown(self, actor: str, reason: str) -> None:
        if not actor.strip() or not reason.strip():
            raise ValueError("force shutdown requires actor and reason")
        now = self.clock.now()
        self._force_shutdown = True
        self._force_actor = actor
        self._force_reason = reason
        self.coordinator.journal.append(
            now, "force-shutdown", actor=actor, reason=reason
        )

    def tick(self) -> SupervisorTick:
        now = self.clock.now()
        captured_control_generation = self._control_generation
        self._drain_health(now)
        self._drain_results(now)
        self._drain_telemetry()
        self.vision.spin_once()
        camera_events = self.vision.drain_events()
        for camera_event in camera_events:
            self.store.put(
                ObservationKey.CAMERA,
                camera_event,
                camera_event.received_at,
                sequence=camera_event.observation_id,
            )
            self.inbox.append(
                Event(
                    camera_event.observation_id,
                    EventType.CAMERA,
                    camera_event.received_at,
                    camera_event,
                    "activity",
                )
            )
        coordinator_events = self.inbox.consume("coordinator")
        inbox_fault = any(
            event.event_type is EventType.OVERFLOW for event in coordinator_events
        )
        activity_events = tuple(
            event.payload
            for event in self.inbox.consume("activity")
            if event.event_type is EventType.CAMERA
        )
        snapshot = self.build_snapshot(
            now,
            control_requested=(
                captured_control_generation > self._processed_control_generation
            ),
            activity_healthy=not inbox_fault,
        )
        status = self.coordinator.update(snapshot, activity_events)
        self._processed_control_generation = max(
            self._processed_control_generation, captured_control_generation
        )
        self._update_led(status)
        safe = self.safe_shutdown_gate(now)
        if now - self._last_status_log >= 1.0:
            print(f"[RUNTIME] {format_status(status, comm_healthy=self.comm_healthy)}")
            self._last_status_log = now
        return SupervisorTick(status, safe, self._force_shutdown)

    def run(self) -> MissionStatus:
        self.install_signal_handlers()
        interval = 1.0 / float(self.config["mission"]["update_rate_hz"])
        latest = self.coordinator.status(self.clock.now())
        while True:
            try:
                tick = self.tick()
            except Exception as exc:
                now = self.clock.now()
                self.coordinator.journal.append(
                    now, "runtime-error", detail=str(exc)
                )
                print(f"[RUNTIME] ERROR: {exc}")
                try:
                    failure_snapshot = self.build_snapshot(
                        now, activity_healthy=False
                    )
                    latest = self.coordinator.update(failure_snapshot)
                except Exception as escalation_error:
                    print(
                        "[RUNTIME] ERROR: could not apply runtime fault policy: "
                        f"{escalation_error}"
                    )
                time.sleep(interval)
                continue
            latest = tick.status
            if tick.force_shutdown:
                break
            if tick.status.terminal and tick.safe_to_shutdown:
                break
            time.sleep(interval)
        self.shutdown()
        return latest

    def build_snapshot(
        self,
        now: float,
        *,
        control_requested: bool = False,
        activity_healthy: bool = True,
    ) -> MissionSnapshot:
        freshness = self.config["freshness_s"]
        heartbeat = self.store.fresh(
            ObservationKey.HEARTBEAT, now, freshness["heartbeat"]
        )
        position = self.store.fresh(
            ObservationKey.LOCAL_POSITION, now, freshness["local_position"]
        )
        attitude = self.store.fresh(
            ObservationKey.ATTITUDE, now, freshness["attitude"]
        )
        ekf = self.store.fresh(ObservationKey.EKF, now, freshness["ekf"])
        landed = self.store.fresh(
            ObservationKey.LANDED_STATE, now, freshness["landed_state"]
        )
        heartbeat_value = heartbeat.value if heartbeat else {}
        return MissionSnapshot(
            now=now,
            comm_healthy=self.comm_healthy,
            heartbeat_fresh=heartbeat is not None,
            mode=heartbeat_value.get("mode") if heartbeat else None,
            armed=heartbeat_value.get("armed") if heartbeat else None,
            ekf_fresh=ekf is not None,
            ekf_healthy=bool(ekf.value) if ekf else None,
            position_fresh=position is not None,
            position=position.value if position else None,
            yaw_fresh=attitude is not None,
            yaw_rad=float(attitude.value) if attitude else None,
            landed_fresh=landed is not None,
            landed_state=landed.value if landed else None,
            operations=self.coordinator.gateway.ledger.snapshot(),
            activity_healthy=(
                activity_healthy and self.coordinator.activities.healthy
            ),
            control_requested=control_requested,
        )

    def safe_shutdown_gate(self, now: float) -> bool:
        max_age = self.config["shutdown"]["grounded_evidence_max_age_s"]
        heartbeat = self.store.fresh(ObservationKey.HEARTBEAT, now, max_age)
        landed = self.store.fresh(ObservationKey.LANDED_STATE, now, max_age)
        return bool(
            heartbeat
            and heartbeat.value.get("armed") is False
            and landed
            and landed.value is LandedState.ON_GROUND
        )

    def shutdown(self) -> None:
        self.stop_event.set()
        if self.comm_process.is_alive():
            self.comm_process.join(timeout=3.0)
        if self.comm_process.is_alive():
            print("[RUNTIME] WARNING: comm process did not stop cleanly")
            self.comm_process.terminate()
            self.comm_process.join(timeout=1.0)
        if self.led is not None:
            self.led.close()
        try:
            self.vision.shutdown()
            self.rclpy.shutdown()
        except Exception:
            pass

    def _drain_health(self, now: float) -> None:
        while True:
            try:
                health: CommHealth = self.health_queue.get_nowait()
            except queue.Empty:
                break
            self.comm_healthy = health.kind is CommHealthKind.HEALTHY
            if health.kind in {CommHealthKind.FAILED, CommHealthKind.STOPPED}:
                self.coordinator.gateway.usable = False
            self.inbox.append(
                Event(
                    f"comm-health/{health.occurred_at}",
                    EventType.COMM_HEALTH,
                    health.occurred_at,
                    health,
                    "coordinator",
                )
            )
        if not self.comm_process.is_alive():
            self.comm_healthy = False
            self.coordinator.gateway.usable = False

    def _drain_results(self, now: float) -> None:
        while True:
            try:
                result: CommResult = self.result_queue.get_nowait()
            except queue.Empty:
                break
            try:
                self.coordinator.gateway.ledger.transition(
                    result.operation_id,
                    result.status,
                    result.occurred_at,
                    producer="comm",
                    attempted_sends=result.attempted_sends,
                    detail=result.detail,
                )
            except (KeyError, ValueError) as exc:
                self.coordinator.journal.append(
                    now,
                    "unmatched-operation-result",
                    operation_id=result.operation_id,
                    status=result.status.value,
                    detail=str(exc),
                )
            self.inbox.append(
                Event(
                    f"operation-result/{result.operation_id}/{result.occurred_at}",
                    EventType.OPERATION_RESULT,
                    result.occurred_at,
                    result,
                    "coordinator",
                )
            )

    def _drain_telemetry(self) -> None:
        latest = None
        while True:
            try:
                latest = self.telemetry_queue.get_nowait()
            except queue.Empty:
                break
        if latest is None:
            return
        packet = (
            latest
            if isinstance(latest, TelemetryPacket)
            else TelemetryPacket(latest, self.clock.now())
        )
        values = packet.values
        heartbeat_time = float(values.get("last_heartbeat_time", 0.0))
        if heartbeat_time:
            try:
                mode = FlightMode(values.get("mode", "UNKNOWN"))
            except ValueError:
                mode = FlightMode.UNKNOWN
            self.store.put(
                ObservationKey.HEARTBEAT,
                {"mode": mode, "armed": bool(values.get("armed", False))},
                heartbeat_time,
            )
        position_time = float(values.get("last_local_position_time", 0.0))
        if position_time:
            self.store.put(
                ObservationKey.LOCAL_POSITION,
                LocalNed(
                    float(values["pos_x_m"]),
                    float(values["pos_y_m"]),
                    float(values["pos_z_m"]),
                ),
                position_time,
            )
        attitude_time = float(values.get("last_attitude_time", 0.0))
        if attitude_time:
            self.store.put(
                ObservationKey.ATTITUDE,
                float(values["yaw_rad"]),
                attitude_time,
            )
        ekf_time = float(values.get("last_ekf_time", 0.0))
        if ekf_time:
            self.store.put(
                ObservationKey.EKF, bool(values.get("ekf_healthy", False)), ekf_time
            )
        landed_time = float(values.get("last_landed_state_time", 0.0))
        if landed_time:
            try:
                landed = LandedState(values.get("landed_state", "undefined"))
            except ValueError:
                landed = LandedState.UNDEFINED
            self.store.put(ObservationKey.LANDED_STATE, landed, landed_time)

    def _update_led(self, status: MissionStatus) -> None:
        if self.led is None:
            return
        autonomous = status.phase in {
            BroadState.TAKEOFF,
            BroadState.SEARCH,
            BroadState.PRECISION_LANDING,
            BroadState.LAND_HERE,
        }
        self.led.update_state(autonomous)

    def _handle_sigint(self, signum, frame) -> None:
        print("\n[RUNTIME] Ctrl+C received: yielding mission control; no LAND issued")
        self.request_control_yield()
