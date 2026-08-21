import math

import pytest

from src.navigation.ned import BodyFrdDisplacement, LocalNed, resolve_body_frd_endpoint


@pytest.mark.parametrize(
    ("yaw", "expected"),
    [
        (0.0, LocalNed(2, 2, -2)),
        (math.pi / 2, LocalNed(1, 3, -2)),
        (-math.pi / 2, LocalNed(1, 1, -2)),
        (math.pi, LocalNed(0, 2, -2)),
    ],
)
def test_forward_transform_at_cardinal_yaws(yaw, expected):
    endpoint = resolve_body_frd_endpoint(
        LocalNed(1, 2, -2), yaw, BodyFrdDisplacement(1, 0, 0)
    )
    assert endpoint.north_m == pytest.approx(expected.north_m)
    assert endpoint.east_m == pytest.approx(expected.east_m)
    assert endpoint.down_m == pytest.approx(expected.down_m)


def test_ned_down_sign_is_preserved():
    endpoint = resolve_body_frd_endpoint(
        LocalNed(0, 0, -2), 0, BodyFrdDisplacement(0, 1, 0.5)
    )
    assert endpoint == LocalNed(0, 1, -1.5)
