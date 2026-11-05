"""Steer-to-Center (Razzaque 2005; Hodgson & Bachmann 2013)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from rdw.controllers._steering import SteeringGoal, SteeringRedirector, goal_toward
from rdw.math2d import Vec2, add, angle_of, direction, scale, subtract, wrap_angle
from rdw.models import Observation


@dataclass
class SteerToCenter(SteeringRedirector):
    """Steer the user toward the center of the physical space.

    When the center is almost straight behind the user, the sign of the
    heading error flips easily and steering would oscillate. In that case a
    temporary target is placed to the side of the user, on the side of the
    center, and kept until the center is in front again.
    """

    temp_target_bearing: float = math.radians(160.0)
    temp_target_distance: float = 4.0

    _temp_target: Vec2 | None = field(default=None, init=False, repr=False)

    @property
    def name(self) -> str:
        return "s2c"

    def reset_goal_state(self) -> None:
        self._temp_target = None

    def steering_goal(self, observation: Observation) -> SteeringGoal | None:
        center = observation.space.center
        position = observation.physical_position
        heading = observation.physical_heading
        bearing = wrap_angle(angle_of(subtract(center, position)) - heading)

        if abs(bearing) >= self.temp_target_bearing:
            if self._temp_target is None:
                side = 1.0 if bearing >= 0 else -1.0
                offset = scale(direction(heading + side * math.pi / 2), self.temp_target_distance)
                self._temp_target = add(position, offset)
            return goal_toward(observation, self._temp_target)

        self._temp_target = None
        return goal_toward(observation, center)


CONTROLLER_FACTORY = SteerToCenter
