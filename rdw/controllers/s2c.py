from __future__ import annotations

from dataclasses import dataclass

from rdw.controllers.base import Redirector
from rdw.math2d import angle_of, subtract, wrap_angle
from rdw.models import Observation, RedirectionCommand


@dataclass
class SteerToCenter(Redirector):
    """Proportional Steer-to-Center baseline."""

    heading_gain: float = 0.8
    max_abs_curvature: float = 0.2

    @property
    def name(self) -> str:
        return "s2c"

    def reset(self) -> None:
        pass

    def compute(self, observation: Observation) -> RedirectionCommand:
        to_center = subtract(observation.space.center, observation.physical_position)
        target_heading = angle_of(to_center)
        heading_error = wrap_angle(target_heading - observation.physical_heading)
        curvature = self.heading_gain * heading_error
        curvature = max(-self.max_abs_curvature, min(curvature, self.max_abs_curvature))
        return RedirectionCommand(curvature=curvature)


CONTROLLER_FACTORY = SteerToCenter
