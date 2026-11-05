from __future__ import annotations

import math
import random

from rdw.math2d import Vec2
from rdw.models import AxisAlignedSpace, Scenario


def benchmark_rooms() -> dict[str, AxisAlignedSpace]:
    """Physical spaces used by the multi-seed random-path benchmark."""
    return {
        "square_6x6": AxisAlignedSpace(-3.0, 3.0, -3.0, 3.0),
        "narrow_4x8": AxisAlignedSpace(-2.0, 2.0, -4.0, 4.0),
        "large_10x10": AxisAlignedSpace(-5.0, 5.0, -5.0, 5.0),
    }


def random_walk_path(
    seed: int,
    min_total_length: float,
    min_segment_length: float = 2.0,
    max_segment_length: float = 8.0,
) -> tuple[Vec2, ...]:
    """Straight virtual segments joined by uniformly random turns.

    Segments are added until the path is at least ``min_total_length`` long.
    The same seed always gives the same path.
    """
    rng = random.Random(seed)
    heading = rng.uniform(-math.pi, math.pi)
    points: list[Vec2] = [(0.0, 0.0)]
    total_length = 0.0
    while total_length < min_total_length or len(points) < 2:
        segment_length = rng.uniform(min_segment_length, max_segment_length)
        x, y = points[-1]
        points.append(
            (x + segment_length * math.cos(heading), y + segment_length * math.sin(heading))
        )
        total_length += segment_length
        heading += rng.uniform(-math.pi, math.pi)
    return tuple(points)


def random_path_scenarios(
    space: AxisAlignedSpace,
    trials: int,
    duration_seconds: float,
    seed: int = 0,
    walking_speed: float = 1.2,
    name_prefix: str = "random",
) -> tuple[Scenario, ...]:
    """One scenario per seed ``seed, seed + 1, ...``; paths outlast the run."""
    # Turning at waypoints costs a little distance, so leave some slack.
    path_length = 1.2 * walking_speed * duration_seconds
    return tuple(
        Scenario(
            name=f"{name_prefix}_{seed + trial:04d}",
            space=space,
            virtual_waypoints=random_walk_path(seed + trial, path_length),
            walking_speed=walking_speed,
            loop_path=False,
        )
        for trial in range(trials)
    )


def default_scenarios() -> tuple[Scenario, ...]:
    square_path = ((0.0, 0.0), (12.0, 0.0), (12.0, 12.0), (0.0, 12.0))
    zigzag_path = (
        (0.0, 0.0),
        (10.0, 3.0),
        (2.0, 7.0),
        (11.0, 11.0),
        (1.0, 15.0),
    )
    return (
        Scenario(
            name="square_room",
            space=AxisAlignedSpace(-3.0, 3.0, -3.0, 3.0),
            virtual_waypoints=square_path,
        ),
        Scenario(
            name="narrow_room",
            space=AxisAlignedSpace(-2.0, 2.0, -4.0, 4.0),
            virtual_waypoints=zigzag_path,
        ),
    )
