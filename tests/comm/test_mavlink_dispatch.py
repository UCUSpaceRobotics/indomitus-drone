import pytest

from src.commands.ledger import OperationStatus
from src.commands.types import (
    Arm,
    LandHere,
    LandingTarget,
    MoveToLocalNed,
    PrecisionLand,
    SetMode,
    Takeoff,
    FlightMode,
)
from src.comm.mavlink_node import CommLoopConfig, dispatch_envelope, stale_envelope_result
from src.comm.messages import CommandEnvelope


class FakeClient:
    def __init__(self, fail=False):
        self.calls = []
        self.fail = fail

    def __getattr__(self, name):
        def call(*args, **kwargs):
            self.calls.append((name, args, kwargs))
            if self.fail:
                raise OSError("uart")
        return call


@pytest.mark.parametrize(
    ("command", "method"),
    [
        (SetMode("mode", FlightMode.GUIDED), "set_mode"),
        (Arm("arm"), "arm"),
        (Takeoff("takeoff", 2), "takeoff"),
        (MoveToLocalNed("move", 1, 2, -2), "send_local_ned_position_target"),
        (PrecisionLand("precision"), "precision_land"),
        (LandHere("land"), "land_here"),
        (LandingTarget("target", 1, 2, 3, "o", 0), "send_landing_target"),
    ],
)
def test_typed_envelope_performs_exactly_one_send_attempt(command, method):
    client = FakeClient()
    result = dispatch_envelope(client, CommandEnvelope(command.operation_id, command, 0), 1)
    assert result.status is OperationStatus.DISPATCHED
    assert result.attempted_sends == 1
    assert [call[0] for call in client.calls] == [method]


def test_send_exception_is_one_attempt_transport_failure():
    client = FakeClient(fail=True)
    command = Arm("arm")
    result = dispatch_envelope(client, CommandEnvelope("arm", command, 0), 1)
    assert result.status is OperationStatus.TRANSPORT_FAILED
    assert result.attempted_sends == 1
    assert len(client.calls) == 1


def test_heartbeat_rate_below_one_hz_is_rejected():
    with pytest.raises(ValueError):
        CommLoopConfig(heartbeat_rate_hz=0.99)


def test_stale_envelope_drops_without_low_level_send():
    client = FakeClient()
    command = Arm("arm")
    envelope = CommandEnvelope("arm", command, 0)
    result = stale_envelope_result(envelope, 0.50001, 0.5)
    assert result.status is OperationStatus.DROPPED
    assert result.attempted_sends == 0
    assert client.calls == []
    assert stale_envelope_result(envelope, 0.5, 0.5) is None


def test_mismatched_operation_id_drops_without_send():
    client = FakeClient()
    command = Arm("command-id")
    result = dispatch_envelope(client, CommandEnvelope("envelope-id", command, 0), 0)
    assert result.status is OperationStatus.DROPPED
    assert result.attempted_sends == 0
    assert client.calls == []


def test_explicit_client_rejection_is_zero_send_failure():
    class RejectingClient(FakeClient):
        def set_mode(self, mode):
            self.calls.append(("set_mode", (mode,), {}))
            return False

    client = RejectingClient()
    command = SetMode("mode", FlightMode.GUIDED)
    result = dispatch_envelope(client, CommandEnvelope("mode", command, 0), 0)
    assert result.status is OperationStatus.TRANSPORT_FAILED
    assert result.attempted_sends == 0
