from rdw.controllers.none import NoRedirection
from rdw.controllers.s2c import SteerToCenter
from rdw.experiments import run_benchmark, save_results
from rdw.models import AxisAlignedSpace, Scenario, SimulationConfig


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
