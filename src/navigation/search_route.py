"""Validated fixed relative route and progress."""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.navigation.ned import BodyFrdDisplacement, LocalNed, resolve_body_frd_endpoint


@dataclass(frozen=True)
class RouteLeg:
    index: int
    displacement: BodyFrdDisplacement


class SearchRoute:
    def __init__(
        self,
        displacements: list[BodyFrdDisplacement] | tuple[BodyFrdDisplacement, ...],
        departure_threshold_m: float,
        position_tolerance_m: float,
    ):
        if not displacements:
            raise ValueError("search route must contain at least one leg")
        if not math.isfinite(departure_threshold_m) or departure_threshold_m <= 0:
            raise ValueError("departure threshold must be positive and finite")
        if not math.isfinite(position_tolerance_m) or position_tolerance_m <= 0:
            raise ValueError("position tolerance must be positive and finite")
        for displacement in displacements:
            if not all(
                math.isfinite(value)
                for value in (
                    displacement.forward_m,
                    displacement.right_m,
                    displacement.down_m,
                )
            ):
                raise ValueError("route displacement must be finite")
            if displacement.norm_m <= departure_threshold_m:
                raise ValueError("every route leg must exceed departure threshold")
            if displacement.norm_m - position_tolerance_m <= departure_threshold_m:
                raise ValueError(
                    "arrival tolerance overlaps mandatory departure threshold"
                )
        self._legs = tuple(RouteLeg(i, value) for i, value in enumerate(displacements))

    def __len__(self) -> int:
        return len(self._legs)

    def leg(self, index: int) -> RouteLeg:
        return self._legs[index]

    def resolve(self, index: int, start: LocalNed, yaw_rad: float) -> LocalNed:
        return resolve_body_frd_endpoint(start, yaw_rad, self.leg(index).displacement)
