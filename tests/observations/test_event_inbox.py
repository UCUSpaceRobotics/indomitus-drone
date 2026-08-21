from src.observations.event_inbox import EventInbox
from src.observations.model import Event, EventType


def test_events_have_one_owner_and_overflow_is_visible():
    inbox = EventInbox(16)
    for index in range(17):
        inbox.append(Event(str(index), EventType.CAMERA, index, index, "activity"))
    inbox.append(Event("control", EventType.CONTROL, 18, None, "coordinator"))

    activity = inbox.consume("activity")
    coordinator = inbox.consume("coordinator")

    assert len(activity) == 15
    assert any(event.event_type is EventType.CONTROL for event in coordinator)
    assert any(event.event_type is EventType.OVERFLOW for event in coordinator)
    assert inbox.overflow_count == 2


def test_overflow_event_emits_once_per_new_overflow_generation():
    inbox = EventInbox(16)
    for index in range(17):
        inbox.append(Event(str(index), EventType.CAMERA, index, index, "activity"))
    first = inbox.consume("coordinator")
    second = inbox.consume("coordinator")
    assert [event.event_type for event in first] == [EventType.OVERFLOW]
    assert second == ()
