import importlib
import sys
import types


class Recorder:
    def __init__(self):
        self.calls = []

    def command_long_send(self, *args):
        self.calls.append(("command_long_send", args))

    def set_position_target_local_ned_send(self, *args):
        self.calls.append(("set_position_target_local_ned_send", args))

    def landing_target_send(self, *args):
        self.calls.append(("landing_target_send", args))


def load_client_module(monkeypatch):
    constants = types.SimpleNamespace(
        MAV_CMD_NAV_LAND=21,
        MAV_CMD_COMPONENT_ARM_DISARM=400,
        PRECISION_LAND_MODE_OPPORTUNISTIC=1,
        PRECISION_LAND_MODE_DISABLED=0,
        MAV_FRAME_LOCAL_NED=1,
        MAV_FRAME_BODY_FRD=12,
        LANDING_TARGET_TYPE_VISION_FIDUCIAL=2,
        MAV_MODE_FLAG_SAFETY_ARMED=128,
        MAV_TYPE_GCS=6,
        MAV_AUTOPILOT_INVALID=8,
    )
    mavutil = types.SimpleNamespace(
        mavlink=constants, mode_string_v10=lambda message: "LOITER"
    )
    package = types.ModuleType("pymavlink")
    package.mavutil = mavutil
    monkeypatch.setitem(sys.modules, "pymavlink", package)
    sys.modules.pop("src.comm.mavlink_client", None)
    return importlib.import_module("src.comm.mavlink_client")


def make_client(module):
    client = module.PixhawkClient.__new__(module.PixhawkClient)
    recorder = Recorder()
    client.connection = types.SimpleNamespace(
        target_system=1, target_component=2, mav=recorder
    )
    return client, recorder


def test_land_param2_distinguishes_precision_and_land_here(monkeypatch):
    module = load_client_module(monkeypatch)
    client, recorder = make_client(module)
    client.precision_land()
    client.land_here()
    assert recorder.calls[0][1][5] == 1
    assert recorder.calls[1][1][5] == 0


def test_absolute_local_ned_and_body_frd_target_encoding(monkeypatch):
    module = load_client_module(monkeypatch)
    client, recorder = make_client(module)
    client.send_local_ned_position_target(1, 2, -3, log=False)
    client.send_landing_target((0.1, -0.2, 2.5))

    position = recorder.calls[0][1]
    assert position[3] == 1
    assert position[5:8] == (1, 2, -3)
    target = recorder.calls[1][1]
    assert target[2] == 12
    assert target[8:11] == (0.1, -0.2, 2.5)
    assert target[-1] == 1


def test_arm_send_is_nonblocking(monkeypatch):
    module = load_client_module(monkeypatch)
    client, recorder = make_client(module)
    assert client.arm(True)
    assert len(recorder.calls) == 1


def test_initial_heartbeat_populates_mode_and_armed(monkeypatch):
    module = load_client_module(monkeypatch)
    client = module.PixhawkClient.__new__(module.PixhawkClient)
    client.telemetry = {
        "connected": False,
        "armed": False,
        "mode": "UNKNOWN",
        "last_heartbeat_time": 0.0,
    }
    message = types.SimpleNamespace(
        base_mode=128,
        type=2,
        autopilot=3,
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 2,
    )
    client.connection = types.SimpleNamespace(
        target_system=1,
        target_component=2,
        recv_match=lambda **kwargs: message,
    )
    assert client.wait_for_heartbeat()
    assert client.telemetry["mode"] == "LOITER"
    assert client.telemetry["armed"] is True


def test_heartbeat_and_ack_sources_are_validated(monkeypatch):
    module = load_client_module(monkeypatch)
    client, _ = make_client(module)
    foreign = types.SimpleNamespace(
        get_srcSystem=lambda: 42, get_srcComponent=lambda: 1
    )
    wrong_component = types.SimpleNamespace(
        get_srcSystem=lambda: 1, get_srcComponent=lambda: 99
    )
    target = types.SimpleNamespace(
        get_srcSystem=lambda: 1, get_srcComponent=lambda: 2
    )
    assert not client._message_from_target(foreign, "HEARTBEAT")
    assert not client._message_from_target(wrong_component, "COMMAND_ACK")
    assert client._message_from_target(target, "HEARTBEAT")
