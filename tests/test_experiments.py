from rdw.controllers.none import NoRedirection
from rdw.controllers.s2c import SteerToCenter
from rdw.experiments import (
    run_benchmark,
    run_random_benchmark,
    save_random_benchmark,
    save_results,
)
from rdw.models import AxisAlignedSpace, Scenario, SimulationConfig


def test_random_benchmark_pairs_seeds_and_writes_outputs(tmp_path) -> None:
    rooms = {
        "small": AxisAlignedSpace(-2.0, 2.0, -2.0, 2.0),
        "large": AxisAlignedSpace(-4.0, 4.0, -4.0, 4.0),
    }
    benchmark = run_random_benchmark(
        rooms,
        [NoRedirection, SteerToCenter],
        trials=3,
        config=SimulationConfig(duration_seconds=5.0),
        seed=7,
    )

    assert len(benchmark.runs) == 2 * 2 * 3
    assert {run["seed"] for run in benchmark.runs} == {7, 8, 9}
    assert len(benchmark.examples) == 2 * 2

    aggregate = save_random_benchmark(benchmark, tmp_path)

    assert len(aggregate) == 2 * 2
    assert all(row["trials"] == 3 for row in aggregate)
    none_rows = [row for row in aggregate if row["controller"] == "none"]
    assert all(row["reset_reduction_vs_none"] == 0.0 for row in none_rows)
    for name in ("random_runs.csv", "random_aggregate.csv", "random_examples.png"):
        assert (tmp_path / name).is_file()


def test_benchmark_writes_summary_trajectories_and_plot(tmp_path) -> None:
    scenario = Scenario(
        name="test",
        space=AxisAlignedSpace(-2.0, 2.0, -2.0, 2.0),
        virtual_waypoints=((0.0, 0.0), (4.0, 0.0)),
    )
    results = run_benchmark(
        [scenario],
        [NoRedirection, SteerToCenter],
        SimulationConfig(duration_seconds=0.5),
    )

    save_results(results, tmp_path)

    assert (tmp_path / "summary.csv").is_file()
    assert (tmp_path / "comparison.png").is_file()
    assert len(list((tmp_path / "trajectories").glob("*.csv"))) == 2
