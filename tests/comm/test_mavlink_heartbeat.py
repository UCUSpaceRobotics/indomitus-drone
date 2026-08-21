from src.comm.mavlink_node import service_due_heartbeat


class HeartbeatClient:
    def __init__(self):
        self.sent = 0

    def send_gcs_heartbeat(self):
        self.sent += 1


def test_heartbeat_remains_due_independently_of_command_load():
    client = HeartbeatClient()
    last = 0.0
    # Simulated busy loop: heartbeat service still runs before each dispatch budget.
    for index in range(1, 201):
        now = index * 0.01
        last = service_due_heartbeat(client, now, last, 0.5)
    assert client.sent >= 3
    assert 2.0 - last < 0.5
