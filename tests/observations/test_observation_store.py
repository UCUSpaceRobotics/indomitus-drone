import pytest

from src.observations.model import ObservationKey
from src.observations.store import ObservationStore


def test_freshness_boundary_is_inclusive_and_clock_is_caller_owned():
    store = ObservationStore()
    store.put(ObservationKey.EKF, True, 10.0)
    assert store.fresh(ObservationKey.EKF, 10.5, 0.5) is not None
    assert store.fresh(ObservationKey.EKF, 10.50001, 0.5) is None
    with pytest.raises(ValueError):
        store.fresh(ObservationKey.EKF, 9.0, 0.5)


def test_receive_time_regression_is_rejected():
    store = ObservationStore()
    store.put(ObservationKey.EKF, True, 2.0)
    with pytest.raises(ValueError):
        store.put(ObservationKey.EKF, False, 1.0)
