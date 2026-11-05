from rdw.controllers.none import NoRedirection
from rdw.controllers.s2c import SteerToCenter
from rdw.models import AxisAlignedSpace, Scenario, SimulationConfig
from rdw.simulation import Simulator


def test_simulator_accepts_replaceable_controllers() -> None:
    scenario = Scenario(
        name="test",
        space=AxisAlignedSpace(-3.0, 3.0, -3.0, 3.0),
        virtual_waypoints=((0.0, 0.0), (10.0, 0.0)),
    )
    simulator = Simulator(SimulationConfig(dt=0.1, duration_seconds=2.0))

    none_result = simulator.run(scenario, NoRedirection())
    s2c_result = simulator.run(scenario, SteerToCenter())

    assert len(none_result.records) == 20
    assert len(s2c_result.records) == 20
    assert none_result.controller_name == "none"
    assert s2c_result.controller_name == "s2c"


def test_metrics_are_complete_and_finite() -> None:
    scenario = Scenario(
        name="test",
        space=AxisAlignedSpace(-2.0, 2.0, -2.0, 2.0),
        virtual_waypoints=((0.0, 0.0), (10.0, 0.0)),
    )
    result = Simulator(SimulationConfig(duration_seconds=1.0)).run(
        scenario,
        SteerToCenter(),
    )

    metrics = result.metrics()
    assert metrics["scenario"] == "test"
    assert metrics["virtual_distance"] > 0
    assert metrics["mean_controller_time_ms"] >= 0
