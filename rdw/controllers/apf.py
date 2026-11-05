"""Artificial Potential Field redirection.

Single-user version of the approach of Thomas & Rosenberg (2019) and
Bachmann et al. (2019): every boundary segment repels the user, and the user
is steered toward the direction of the total repulsive force. The steering
itself uses the same curvature / rotation gain rule as S2C and S2O.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from rdw.controllers._steering import SteeringGoal, SteeringRedirector
from rdw.math2d import Vec2, add, angle_of, length, scale, subtract
from rdw.models import AxisAlignedSpace, Observation

Segment = tuple[Vec2, Vec2]


def wall_segments(space: AxisAlignedSpace) -> tuple[Segment, ...]:
    """Room walls as segments. Obstacles will add their own segments here."""
    corners = (
        (space.min_x, space.min_y),
        (space.max_x, space.min_y),
        (space.max_x, space.max_y),
        (space.min_x, space.max_y),
    )
    return tuple((corners[i], corners[(i + 1) % 4]) for i in range(4))


def closest_point_on_segment(point: Vec2, segment: Segment) -> Vec2:
    start, end = segment
    edge = subtract(end, start)
    edge_length_sq = edge[0] ** 2 + edge[1] ** 2
    if edge_length_sq == 0.0:
        return start
    to_point = subtract(point, start)
    t = (to_point[0] * edge[0] + to_point[1] * edge[1]) / edge_length_sq
    return add(start, scale(edge, max(0.0, min(t, 1.0))))


def repulsive_force(
    point: Vec2,
    segments: tuple[Segment, ...],
    piece_length: float = 0.25,
    min_distance: float = 0.05,
) -> Vec2:
    """Sum of ``piece_length * unit(point - q) / distance**2`` over boundary pieces.

    Each segment is split into pieces no longer than ``piece_length`` and q is
    the closest point of a piece. For a long straight wall this field falls
    off as 1 / distance, so near walls dominate but far walls still count.
    """
    force_x = 0.0
    force_y = 0.0
    for start, end in segments:
        edge = subtract(end, start)
        pieces = max(1, math.ceil(length(edge) / piece_length))
        actual_piece_length = length(edge) / pieces
        for i in range(pieces):
            piece = (
                add(start, scale(edge, i / pieces)),
                add(start, scale(edge, (i + 1) / pieces)),
            )
            away = subtract(point, closest_point_on_segment(point, piece))
            distance = max(length(away), min_distance)
            weight = actual_piece_length / distance**3
            force_x += away[0] * weight
            force_y += away[1] * weight
    return force_x, force_y


@dataclass
class ArtificialPotentialField(SteeringRedirector):
    piece_length: float = 0.25
    min_distance: float = 0.05
    min_force: float = 1e-6

    @property
    def name(self) -> str:
        return "apf"

    def force(self, observation: Observation) -> Vec2:
        return repulsive_force(
            observation.physical_position,
            wall_segments(observation.space),
            self.piece_length,
            self.min_distance,
        )

    def steering_goal(self, observation: Observation) -> SteeringGoal | None:
        force = self.force(observation)
        if length(force) < self.min_force:
            return None
        return SteeringGoal(heading=angle_of(force))


CONTROLLER_FACTORY = ArtificialPotentialField
