import queue

from src.commands.ledger import OperationStatus
from src.comm.mavlink_node import _process_protocol_events


def test_ack_is_correlated_once_and_unmatched_ack_is_ignored():
    results = queue.Queue()
    outstanding = {400: ("arm", 4.0)}
    _process_protocol_events(
        [
            {
                "type": "command_ack",
                "command": 999,
                "result": 0,
                "accepted": True,
                "received_at": 1.0,
            },
            {
                "type": "command_ack",
                "command": 400,
                "result": 0,
                "accepted": True,
                "received_at": 1.1,
            },
        ],
        outstanding,
        results,
    )
    result = results.get_nowait()
    assert result.operation_id == "arm"
    assert result.status is OperationStatus.ACKNOWLEDGED
    assert outstanding == {}


def test_negative_ack_is_rejected():
    results = queue.Queue()
    outstanding = {21: ("land", 4.0)}
    _process_protocol_events(
        [
            {
                "type": "command_ack",
                "command": 21,
                "result": 4,
                "accepted": False,
                "received_at": 2.0,
            }
        ],
        outstanding,
        results,
    )
    assert results.get_nowait().status is OperationStatus.REJECTED


def test_in_progress_ack_keeps_correlation_open():
    results = queue.Queue()
    outstanding = {21: ("land", 4.0)}
    _process_protocol_events(
        [
            {
                "type": "command_ack",
                "command": 21,
                "result": 5,
                "accepted": False,
                "received_at": 2.0,
            }
        ],
        outstanding,
        results,
    )
    assert outstanding == {21: ("land", 4.0)}
    assert results.empty()
