import math
import random

import pytest

from rdw.controllers import discover_controllers
from rdw.controllers.apf import ArtificialPotentialField, repulsive_force, wall_segments
from rdw.controllers.s2c import SteerToCenter
from rdw.controllers.s2o import SteerToOrbit
from rdw.experiments import aggregate_runs, run_random_benchmark
from rdw.math2d import angle_of, length, subtract, wrap_angle
from rdw.models import AxisAlignedSpace, ControlLimits, Observation, SimulationConfig

SQUARE = AxisAlignedSpace(-3.0, 3.0, -3.0, 3.0)
NARROW = AxisAlignedSpace(-2.0, 2.0, -4.0, 4.0)


def observe(
    position: tuple[float, float],
    heading: float,
    virtual_heading: float = 0.0,
    space: AxisAlignedSpace = SQUARE,
    step: int = 0,
) -> Observation:
    return Observation(
        step=step,
        dt=0.05,
        physical_position=position,
        physical_heading=heading,
        virtual_position=(0.0, 0.0),
        virtual_heading=virtual_heading,
        walking_speed=1.2,
        space=space,
    )


def within(command, limits: ControlLimits) -> bool:
    return command == command.clamped(limits)


@pytest.mark.parametrize("name", ["s2c", "s2o", "apf"])
def test_commands_stay_within_shared_limits(name: str) -> None:
    rng = random.Random(0)
    controller = discover_controllers()[name]()
    controller.reset()
    virtual_heading = 0.0
    for step in range(300):
        virtual_heading = wrap_angle(virtual_heading + rng.uniform(-0.1, 0.1))
        obs = observe(
            (rng.uniform(-2.9, 2.9), rng.uniform(-2.9, 2.9)),
            rng.uniform(-math.pi, math.pi),
            virtual_heading,
            step=step,
        )
        assert within(controller.compute(obs), obs.limits)


def test_rotation_gain_amplifies_turns_toward_the_goal() -> None:
    # The center is to the user's left.
    toward = SteerToCenter()
    toward.compute(observe((2.0, 0.0), math.pi / 2, virtual_heading=0.0))
    command = toward.compute(observe((2.0, 0.0), math.pi / 2, virtual_heading=0.075, step=1))
    assert command.rotation_gain < 1.0
    assert command.curvature == 0.0

    away = SteerToCenter()
    away.compute(observe((2.0, 0.0), math.pi / 2, virtual_heading=0.0))
    command = away.compute(observe((2.0, 0.0), math.pi / 2, virtual_heading=-0.075, step=1))
    assert command.rotation_gain > 1.0


def test_s2c_keeps_turning_one_way_when_center_is_behind() -> None:
    controller = SteerToCenter()
    # Center slightly to the left of straight behind: steer left ...
    first = controller.compute(observe((2.0, 0.0), 0.1))
    # ... and keep steering left after the center crosses to the right side,
    # because the temporary target is kept while the center stays behind.
    second = controller.compute(observe((2.0, 0.0), -0.05, step=1))
    assert first.curvature > 0
    assert second.curvature > 0

    goal = controller.steering_goal(observe((2.0, 0.0), math.pi / 2))
    assert goal.heading == pytest.approx(math.pi)  # center in front again: target the center


def test_s2o_uses_the_tangent_point_ahead_outside_the_orbit() -> None:
    controller = SteerToOrbit()
    obs = observe((2.5, 0.0), math.pi / 2)
    radius = controller.radius_for(SQUARE)
    goal = controller.steering_goal(obs)
    target = (
        2.5 + goal.distance * math.cos(goal.heading),
        goal.distance * math.sin(goal.heading),
    )

    assert length(target) == pytest.approx(radius)
    assert target[1] > 0  # the tangent point in front of a north-facing user
    # A tangent point: the radius is perpendicular to the line of sight.
    to_user = subtract((2.5, 0.0), target)
    assert target[0] * to_user[0] + target[1] * to_user[1] == pytest.approx(0.0, abs=1e-9)


def test_s2o_targets_points_on_the_orbit_from_inside() -> None:
    controller = SteerToOrbit()
    obs = observe((0.5, 0.0), math.pi / 2)
    radius = controller.radius_for(SQUARE)
    targets = controller.orbit_targets(obs)

    for target in targets:
        assert length(target) == pytest.approx(radius)
        assert abs(wrap_angle(angle_of(target))) == pytest.approx(controller.target_angle)


def test_apf_force_points_away_from_walls() -> None:
    near_left_wall = repulsive_force((-1.5, 0.0), wall_segments(NARROW))
    assert near_left_wall[0] > 0
    assert abs(near_left_wall[1]) < 1e-9

    near_corner = repulsive_force((2.5, 2.5), wall_segments(SQUARE))
    assert angle_of(near_corner) == pytest.approx(-3 * math.pi / 4)

    centered = repulsive_force((0.0, 0.0), wall_segments(SQUARE))
    assert length(centered) < 1e-9


def test_apf_steers_toward_the_force() -> None:
    controller = ArtificialPotentialField()
    # Near the left wall of the narrow room, facing north: turn right.
    command = controller.compute(observe((-1.5, 0.0), math.pi / 2, space=NARROW))
    assert command.curvature < 0

    controller.reset()
    command = controller.compute(observe((0.0, 0.0), math.pi / 2))
    assert command.curvature == 0.0


def test_baselines_reset_less_than_no_redirection() -> None:
    controllers = discover_controllers()
    benchmark = run_random_benchmark(
        {"large": AxisAlignedSpace(-5.0, 5.0, -5.0, 5.0)},
        [controllers[name] for name in ("none", "s2c", "s2o", "apf")],
        trials=8,
        config=SimulationConfig(duration_seconds=60.0),
    )
    reduction = {
        row["controller"]: row["reset_reduction_vs_none"]
        for row in aggregate_runs(benchmark.runs)
    }

    for name in ("s2c", "s2o", "apf"):
        assert reduction[name] > 0, (name, reduction)
