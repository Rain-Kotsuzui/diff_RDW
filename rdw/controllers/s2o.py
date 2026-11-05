"""Steer-to-Orbit (Hodgson & Bachmann 2013)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from rdw.controllers._steering import SteeringGoal, SteeringRedirector, goal_toward
from rdw.math2d import Vec2, add, angle_of, direction, length, scale, subtract, wrap_angle
from rdw.models import AxisAlignedSpace, Observation


@dataclass
class SteerToOrbit(SteeringRedirector):
    """Steer the user onto a circle around the center of the physical space.

    Outside the circle the two tangent points are candidate targets. Inside
    it, the candidates are the points on the circle ``target_angle`` before
    and after the user's polar angle. The candidate that needs less turning
    is used. The literature uses a 5 m radius for large rooms; by default the
    radius here scales with the room so that it also fits small rooms.
    """

    orbit_radius: float | None = None
    orbit_radius_ratio: float = 0.6
    target_angle: float = math.radians(60.0)

    @property
    def name(self) -> str:
        return "s2o"

    def radius_for(self, space: AxisAlignedSpace) -> float:
        if self.orbit_radius is not None:
            return self.orbit_radius
        half_extent = min(space.max_x - space.min_x, space.max_y - space.min_y) / 2
        return max(self.orbit_radius_ratio * (half_extent - space.safety_margin), 0.1)

    def orbit_targets(self, observation: Observation) -> tuple[Vec2, Vec2]:
        center = observation.space.center
        radius = self.radius_for(observation.space)
        from_center = subtract(observation.physical_position, center)
        distance = length(from_center)
        # At the exact center every polar angle is equivalent; use the heading.
        polar = angle_of(from_center) if distance > 1e-9 else observation.physical_heading

        if distance > radius:
            spread = math.acos(radius / distance)
        else:
            spread = self.target_angle
        return (
            add(center, scale(direction(polar + spread), radius)),
            add(center, scale(direction(polar - spread), radius)),
        )

    def steering_goal(self, observation: Observation) -> SteeringGoal | None:
        position = observation.physical_position
        heading = observation.physical_heading
        target = min(
            self.orbit_targets(observation),
            key=lambda point: abs(wrap_angle(angle_of(subtract(point, position)) - heading)),
        )
        return goal_toward(observation, target)


CONTROLLER_FACTORY = SteerToOrbit
