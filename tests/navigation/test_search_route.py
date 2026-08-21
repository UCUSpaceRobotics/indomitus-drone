import pytest

from src.navigation.ned import BodyFrdDisplacement, LocalNed
from src.navigation.search_route import SearchRoute


def test_route_rejects_zero_short_and_ambiguous_legs():
    with pytest.raises(ValueError):
        SearchRoute([BodyFrdDisplacement(0, 0, 0)], 0.15, 0.2)
    with pytest.raises(ValueError):
        SearchRoute([BodyFrdDisplacement(0.15, 0, 0)], 0.15, 0.01)
    with pytest.raises(ValueError):
        SearchRoute([BodyFrdDisplacement(0.3, 0, 0)], 0.15, 0.2)


def test_endpoint_is_value_frozen_after_resolution():
    route = SearchRoute([BodyFrdDisplacement(1, 0, 0)], 0.15, 0.2)
    endpoint = route.resolve(0, LocalNed(0, 0, -2), 0)
    route.resolve(0, LocalNed(4, 4, -1), 1.0)
    assert endpoint == LocalNed(1, 0, -2)
