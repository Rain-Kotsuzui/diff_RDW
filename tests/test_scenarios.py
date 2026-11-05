from rdw.math2d import length, subtract
from rdw.models import AxisAlignedSpace
from rdw.scenarios import random_path_scenarios, random_walk_path


def path_length(points) -> float:
    return sum(length(subtract(b, a)) for a, b in zip(points, points[1:]))


def test_random_walk_path_is_reproducible_and_long_enough() -> None:
    first = random_walk_path(3, min_total_length=50.0)

    assert first == random_walk_path(3, min_total_length=50.0)
    assert first != random_walk_path(4, min_total_length=50.0)
    assert first[0] == (0.0, 0.0)
    assert path_length(first) >= 50.0
    for a, b in zip(first, first[1:]):
        assert 2.0 <= length(subtract(b, a)) <= 8.0


def test_random_path_scenarios_use_consecutive_seeds() -> None:
    space = AxisAlignedSpace(-3.0, 3.0, -3.0, 3.0)
    scenarios = random_path_scenarios(space, trials=3, duration_seconds=30.0, seed=5, name_prefix="room")

    assert [scenario.name for scenario in scenarios] == ["room_0005", "room_0006", "room_0007"]
    assert scenarios[1].virtual_waypoints == random_walk_path(6, 1.2 * 1.2 * 30.0)
    for scenario in scenarios:
        assert not scenario.loop_path
        assert path_length(scenario.virtual_waypoints) >= scenario.walking_speed * 30.0
