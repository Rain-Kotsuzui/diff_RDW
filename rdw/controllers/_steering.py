"""Shared steering mechanics for the steer-to-X baselines (S2C, S2O, APF).

The update rule follows the ``SteerToRedirector`` of the Redirected Walking
Toolkit (Azmandian et al. 2016), which implements the generalized steering
algorithms of Hodgson & Bachmann (2013):

* Each step injects one extra physical rotation toward a desired heading.
  While the user walks it comes from a curvature gain, and while the user
  turns it comes from a rotation gain; whichever yields more rotation is used.
* The injected rotation is dampened when the user already faces the goal or
  is close to the goal point, and it is low-pass filtered over time.

Simulator conventions: positive curvature turns the physical heading
counter-clockwise, and ``physical_turn = virtual_turn / rotation_gain``.

This module starts with an underscore, so controller discovery skips it.
"""

from __future__ import annotations

import math
from abc import abstractmethod
from dataclasses import dataclass, field

from rdw.controllers.base import Redirector
from rdw.math2d import Vec2, angle_of, length, subtract, wrap_angle
from rdw.models import Observation, RedirectionCommand


@dataclass(frozen=True)
class SteeringGoal:
    """Desired physical heading, plus the distance to the goal point if any."""

    heading: float
    distance: float | None = None


def goal_toward(observation: Observation, target: Vec2) -> SteeringGoal | None:
    offset = subtract(target, observation.physical_position)
    distance = length(offset)
    if distance < 1e-9:
        return None
    return SteeringGoal(heading=angle_of(offset), distance=distance)


@dataclass
class SteeringRedirector(Redirector):
    """Base class: subclasses only decide where the user should be steered."""

    curvature_radius: float = 7.5
    max_curvature_rate: float = math.radians(15.0)
    max_rotation_rate: float = math.radians(30.0)
    rotation_threshold: float = math.radians(1.5)
    bearing_dampening_threshold: float = math.radians(45.0)
    distance_dampening_threshold: float = 1.25
    # The toolkit blends 0.125 of the new value per frame at about 60 Hz,
    # which is a time constant of about 0.125 s.
    smoothing_time_constant: float = 0.125

    _previous_virtual_heading: float | None = field(default=None, init=False, repr=False)
    _last_rotation_rate: float = field(default=0.0, init=False, repr=False)

    @abstractmethod
    def steering_goal(self, observation: Observation) -> SteeringGoal | None:
        """Where to steer the user; ``None`` means no preferred direction."""

    def reset_goal_state(self) -> None:
        """Clear subclass state; called from ``reset``."""

    def reset(self) -> None:
        self._previous_virtual_heading = None
        self._last_rotation_rate = 0.0
        self.reset_goal_state()

    def compute(self, observation: Observation) -> RedirectionCommand:
        dt = observation.dt
        speed = observation.walking_speed
        limits = observation.limits
        virtual_turn = self._estimate_virtual_turn(observation)

        goal = self.steering_goal(observation)
        proposed_rate = 0.0
        use_curvature = True
        if goal is not None:
            heading_error = wrap_angle(goal.heading - observation.physical_heading)
            steer = math.copysign(1.0, heading_error) if heading_error != 0.0 else 0.0

            curvature_rate = min(
                speed / self.curvature_radius,
                self.max_curvature_rate,
                limits.max_abs_curvature * speed,
            )
            rotation_rate = self._rotation_gain_rate(virtual_turn / dt, steer, observation)
            use_curvature = curvature_rate >= rotation_rate
            proposed_rate = steer * max(curvature_rate, rotation_rate)

            bearing = abs(heading_error)
            if bearing < self.bearing_dampening_threshold:
                proposed_rate *= math.sin(0.5 * math.pi * bearing / self.bearing_dampening_threshold)
            if goal.distance is not None and goal.distance < self.distance_dampening_threshold:
                proposed_rate *= goal.distance / self.distance_dampening_threshold

        alpha = 1.0 - math.exp(-dt / self.smoothing_time_constant)
        rate = self._last_rotation_rate + alpha * (proposed_rate - self._last_rotation_rate)
        self._last_rotation_rate = rate

        if use_curvature or virtual_turn == 0.0:
            command = RedirectionCommand(curvature=rate / speed)
        else:
            command = RedirectionCommand(
                rotation_gain=self._rotation_gain_for(rate * dt, virtual_turn, observation)
            )
        return command.clamped(limits)

    def _estimate_virtual_turn(self, observation: Observation) -> float:
        """Virtual turn of the previous step, used as this step's estimate."""
        previous = self._previous_virtual_heading
        self._previous_virtual_heading = observation.virtual_heading
        if previous is None:
            return 0.0
        return wrap_angle(observation.virtual_heading - previous)

    def _rotation_gain_rate(
        self, virtual_turn_rate: float, steer: float, observation: Observation
    ) -> float:
        """Extra physical rotation rate that a rotation gain can inject."""
        if abs(virtual_turn_rate) < self.rotation_threshold or steer == 0.0:
            return 0.0
        limits = observation.limits
        if virtual_turn_rate * steer > 0:
            # The user turns toward the goal: amplify the physical turn.
            factor = 1.0 / limits.min_rotation_gain - 1.0
        else:
            # The user turns away from the goal: shrink the physical turn.
            factor = 1.0 - 1.0 / limits.max_rotation_gain
        return min(abs(virtual_turn_rate) * max(factor, 0.0), self.max_rotation_rate)

    @staticmethod
    def _rotation_gain_for(
        extra_turn: float, virtual_turn: float, observation: Observation
    ) -> float:
        """Rotation gain whose physical turn exceeds ``virtual_turn`` by ``extra_turn``."""
        ratio = 1.0 + extra_turn / virtual_turn
        if ratio <= 0.0:
            return observation.limits.max_rotation_gain
        return 1.0 / ratio
