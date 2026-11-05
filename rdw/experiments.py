from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from rdw.controllers import discover_controllers
from rdw.controllers.registry import ControllerFactory
from rdw.models import AxisAlignedSpace, Scenario, SimulationConfig
from rdw.scenarios import benchmark_rooms, default_scenarios, random_path_scenarios
from rdw.simulation import SimulationResult, Simulator

CONTROLLERS = discover_controllers()

Row = dict[str, float | str]

# Per-run metrics that are averaged over seeds in the random-path benchmark.
AVERAGED_METRICS = (
    "min_boundary_distance",
    "mean_boundary_distance",
    "mean_abs_curvature",
    "mean_controller_time_ms",
)


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


@dataclass(frozen=True)
class RandomBenchmark:
    runs: list[Row]
    """One row of metrics per room, controller and seed."""
    examples: list[SimulationResult]
    """Results of the first seed in every room, for plotting."""


def run_random_benchmark(
    rooms: Mapping[str, AxisAlignedSpace],
    controller_factories: Sequence[ControllerFactory],
    trials: int,
    config: SimulationConfig | None = None,
    seed: int = 0,
) -> RandomBenchmark:
    """Run every controller on the same ``trials`` random paths in every room."""
    config = config or SimulationConfig()
    simulator = Simulator(config)
    runs: list[Row] = []
    examples: list[SimulationResult] = []
    for room, space in rooms.items():
        scenarios = random_path_scenarios(
            space, trials, config.duration_seconds, seed, name_prefix=room
        )
        for trial_seed, scenario in enumerate(scenarios, start=seed):
            for factory in controller_factories:
                result = simulator.run(scenario, factory())
                runs.append({"room": room, "seed": trial_seed, **result.metrics()})
                if trial_seed == seed:
                    examples.append(result)
    return RandomBenchmark(runs, examples)


def aggregate_runs(runs: Iterable[Row]) -> list[Row]:
    """Mean and spread of the metrics per room and controller.

    All controllers run on the same seeds, so ``reset_reduction_vs_none`` is a
    paired comparison against the ``none`` controller of the same room.
    """
    groups: dict[tuple[str, str], list[Row]] = {}
    for run in runs:
        groups.setdefault((str(run["room"]), str(run["controller"])), []).append(run)

    rows: list[Row] = []
    for (room, controller), group in groups.items():
        resets = [float(run["resets"]) for run in group]
        virtual_distance = sum(float(run["virtual_distance"]) for run in group)
        spread = statistics.stdev(resets) if len(resets) > 1 else 0.0
        baseline = groups.get((room, "none"))
        baseline_resets = sum(float(run["resets"]) for run in baseline) if baseline else 0.0
        row: Row = {
            "room": room,
            "controller": controller,
            "trials": len(group),
            "resets_mean": statistics.fmean(resets),
            "resets_std": spread,
            "resets_sem": spread / math.sqrt(len(resets)),
            "resets_per_100m": (
                100.0 * sum(resets) / virtual_distance if virtual_distance > 0 else math.nan
            ),
            "reset_reduction_vs_none": (
                1.0 - sum(resets) / baseline_resets if baseline_resets > 0 else math.nan
            ),
        }
        for name in AVERAGED_METRICS:
            row[f"{name}_mean"] = statistics.fmean(float(run[name]) for run in group)
        rows.append(row)
    return rows


def save_random_benchmark(benchmark: RandomBenchmark, output_dir: Path) -> list[Row]:
    output_dir.mkdir(parents=True, exist_ok=True)
    aggregate = aggregate_runs(benchmark.runs)
    _write_rows(output_dir / "random_runs.csv", benchmark.runs)
    _write_rows(output_dir / "random_aggregate.csv", aggregate)
    _save_comparison_plot(benchmark.examples, output_dir / "random_examples.png")
    return aggregate


def _write_rows(path: Path, rows: list[Row]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def save_results(results: list[SimulationResult], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    trajectory_dir = output_dir / "trajectories"
    trajectory_dir.mkdir(exist_ok=True)

    _write_rows(output_dir / "summary.csv", [result.metrics() for result in results])

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
    parser.add_argument(
        "--trials",
        type=int,
        default=0,
        help="Random paths per room. 0 runs the fixed scenarios instead.",
    )
    parser.add_argument("--seed", type=int, default=0, help="First random-path seed.")
    args = parser.parse_args()

    factories = [CONTROLLERS[name] for name in args.controllers]
    config = SimulationConfig(duration_seconds=args.duration)

    if args.trials > 0:
        benchmark = run_random_benchmark(
            benchmark_rooms(), factories, args.trials, config, args.seed
        )
        for row in save_random_benchmark(benchmark, args.output):
            print(
                f"{row['room']:12} {row['controller']:5} "
                f"resets={row['resets_mean']:6.2f} ± {row['resets_sem']:.2f} "
                f"per_100m={row['resets_per_100m']:5.2f} "
                f"vs_none={row['reset_reduction_vs_none']:+.1%}"
            )
        return

    results = run_benchmark(default_scenarios(), factories, config)
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
