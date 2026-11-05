from __future__ import annotations

import argparse
import csv
from collections.abc import Iterable
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from rdw.controllers import discover_controllers
from rdw.controllers.registry import ControllerFactory
from rdw.models import Scenario, SimulationConfig
from rdw.scenarios import default_scenarios
from rdw.simulation import SimulationResult, Simulator

CONTROLLERS = discover_controllers()


def run_benchmark(
    scenarios: Iterable[Scenario],
    controller_factories: Iterable[ControllerFactory],
    config: SimulationConfig | None = None,
) -> list[SimulationResult]:
    simulator = Simulator(config)
    return [
        simulator.run(scenario, factory())
        for scenario in scenarios
        for factory in controller_factories
    ]


def save_results(results: list[SimulationResult], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    trajectory_dir = output_dir / "trajectories"
    trajectory_dir.mkdir(exist_ok=True)

    metrics = [result.metrics() for result in results]
    with (output_dir / "summary.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(metrics[0]))
        writer.writeheader()
        writer.writerows(metrics)

    for result in results:
        path = trajectory_dir / f"{result.scenario_name}__{result.controller_name}.csv"
        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "time",
                    "physical_x",
                    "physical_y",
                    "virtual_x",
                    "virtual_y",
                    "boundary_distance",
                    "curvature",
                    "reset",
                    "outside",
                ]
            )
            for record in result.records:
                writer.writerow(
                    [
                        record.time,
                        *record.physical_position,
                        *record.virtual_position,
                        record.boundary_distance,
                        record.command.curvature,
                        int(record.reset),
                        int(record.outside),
                    ]
                )

    _save_comparison_plot(results, output_dir / "comparison.png")


def _save_comparison_plot(results: list[SimulationResult], output_path: Path) -> None:
    scenario_names = list(dict.fromkeys(result.scenario_name for result in results))
    figure, axes = plt.subplots(1, len(scenario_names), figsize=(6 * len(scenario_names), 5))
    axes = [axes] if len(scenario_names) == 1 else axes

    for axis, scenario_name in zip(axes, scenario_names, strict=True):
        scenario_results = [
            result for result in results if result.scenario_name == scenario_name
        ]
        space = scenario_results[0].space
        axis.add_patch(
            Rectangle(
                (space.min_x, space.min_y),
                space.max_x - space.min_x,
                space.max_y - space.min_y,
                fill=False,
                color="black",
                linewidth=1.5,
                label="room",
            )
        )
        axis.add_patch(
            Rectangle(
                (
                    space.min_x + space.safety_margin,
                    space.min_y + space.safety_margin,
                ),
                space.max_x - space.min_x - 2 * space.safety_margin,
                space.max_y - space.min_y - 2 * space.safety_margin,
                fill=False,
                color="gray",
                linestyle="--",
                linewidth=1.0,
                label="safe area",
            )
        )
        for result in results:
            if result.scenario_name != scenario_name:
                continue
            x = [record.physical_position[0] for record in result.records]
            y = [record.physical_position[1] for record in result.records]
            axis.plot(x, y, label=result.controller_name)
            reset_x = [
                record.physical_position[0] for record in result.records if record.reset
            ]
            reset_y = [
                record.physical_position[1] for record in result.records if record.reset
            ]
            axis.scatter(reset_x, reset_y, marker="x", s=20)
        axis.set_title(scenario_name)
        axis.set_xlabel("physical x (m)")
        axis.set_ylabel("physical y (m)")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.25)
        axis.legend()

    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run redirected-walking baselines.")
    parser.add_argument(
        "--controllers",
        nargs="+",
        choices=CONTROLLERS,
        default=list(CONTROLLERS),
    )
    parser.add_argument("--output", type=Path, default=Path("results"))
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()

    factories = [CONTROLLERS[name] for name in args.controllers]
    results = run_benchmark(
        default_scenarios(),
        factories,
        SimulationConfig(duration_seconds=args.duration),
    )
    save_results(results, args.output)

    for result in results:
        metrics = result.metrics()
        print(
            f"{metrics['scenario']:12} {metrics['controller']:5} "
            f"resets={metrics['resets']:.0f} "
            f"outside_steps={metrics['outside_steps']:.0f} "
            f"min_distance={metrics['min_boundary_distance']:.3f} m"
        )


if __name__ == "__main__":
    main()
