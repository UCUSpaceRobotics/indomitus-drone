"""Local-NED and body-relative route geometry."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LocalNed:
    north_m: float
    east_m: float
    down_m: float

    def distance_to(self, other: "LocalNed") -> float:
        return math.dist(
            (self.north_m, self.east_m, self.down_m),
            (other.north_m, other.east_m, other.down_m),
        )


@dataclass(frozen=True)
class BodyFrdDisplacement:
    forward_m: float
    right_m: float
    down_m: float

    @property
    def norm_m(self) -> float:
        return math.sqrt(
            self.forward_m**2 + self.right_m**2 + self.down_m**2
        )


def resolve_body_frd_endpoint(
    start: LocalNed, yaw_rad: float, displacement: BodyFrdDisplacement
) -> LocalNed:
    values = (
        start.north_m,
        start.east_m,
        start.down_m,
        yaw_rad,
        displacement.forward_m,
        displacement.right_m,
        displacement.down_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("NED transform values must be finite")
    delta_north = (
        displacement.forward_m * math.cos(yaw_rad)
        - displacement.right_m * math.sin(yaw_rad)
    )
    delta_east = (
        displacement.forward_m * math.sin(yaw_rad)
        + displacement.right_m * math.cos(yaw_rad)
    )
    return LocalNed(
        start.north_m + delta_north,
        start.east_m + delta_east,
        start.down_m + displacement.down_m,
    )
