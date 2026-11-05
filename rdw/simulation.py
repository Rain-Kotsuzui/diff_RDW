from __future__ import annotations

import math
from dataclasses import dataclass
from time import perf_counter

from rdw.controllers.base import Redirector
from rdw.math2d import add, angle_of, direction, length, scale, subtract, wrap_angle
from rdw.models import (
    AxisAlignedSpace,
    Observation,
    RedirectionCommand,
    Scenario,
    SimulationConfig,
    StepRecord,
)


@dataclass(frozen=True)
class SimulationResult:
    scenario_name: str
    controller_name: str
    space: AxisAlignedSpace
    records: tuple[StepRecord, ...]

    def metrics(self) -> dict[str, float | str]:
        records = self.records
        return {
            "scenario": self.scenario_name,
            "controller": self.controller_name,
            "resets": float(sum(record.reset for record in records)),
            "outside_steps": float(sum(record.outside for record in records)),
            "min_boundary_distance": min(record.boundary_distance for record in records),
            "mean_boundary_distance": sum(record.boundary_distance for record in records) / len(records),
            "physical_distance": sum(record.physical_distance for record in records),
            "virtual_distance": sum(record.virtual_distance for record in records),
            "mean_abs_curvature": sum(abs(record.command.curvature) for record in records) / len(records),
            "max_abs_curvature": max(abs(record.command.curvature) for record in records),
            "mean_controller_time_ms": sum(record.controller_time_ms for record in records) / len(records),
        }


class Simulator:
    """Algorithm-agnostic single-user redirected-walking simulator."""

    def __init__(self, config: SimulationConfig | None = None) -> None:
        self.config = config or SimulationConfig()

    def run(self, scenario: Scenario, controller: Redirector) -> SimulationResult:
        controller.reset()
        config = self.config
        total_steps = math.ceil(config.duration_seconds / config.dt)

        physical_position = scenario.initial_physical_position
        physical_heading = scenario.initial_physical_heading
        virtual_position = scenario.virtual_waypoints[0]
        virtual_heading = angle_of(subtract(scenario.virtual_waypoints[1], virtual_position))
        waypoint_index = 1
        was_in_danger = False
        records: list[StepRecord] = []

        for step in range(total_steps):
            observation = Observation(
                step=step,
                dt=config.dt,
                physical_position=physical_position,
                physical_heading=physical_heading,
                virtual_position=virtual_position,
                virtual_heading=virtual_heading,
                walking_speed=scenario.walking_speed,
                space=scenario.space,
            )

            start = perf_counter()
            command = controller.compute(observation).clamped(config.limits)
            controller_time_ms = (perf_counter() - start) * 1000.0

            target = scenario.virtual_waypoints[waypoint_index]
            to_target = subtract(target, virtual_position)
            target_distance = length(to_target)
            target_heading = angle_of(to_target)
            virtual_turn = max(
                -config.max_virtual_turn_rate * config.dt,
                min(
                    wrap_angle(target_heading - virtual_heading),
                    config.max_virtual_turn_rate * config.dt,
                ),
            )
            virtual_heading = wrap_angle(virtual_heading + virtual_turn)

            virtual_distance = min(scenario.walking_speed * config.dt, target_distance)
            physical_distance = virtual_distance / command.translation_gain
            physical_heading = wrap_angle(
                physical_heading
                + virtual_turn / command.rotation_gain
                + command.curvature * physical_distance
            )
            virtual_position = add(virtual_position, scale(direction(virtual_heading), virtual_distance))
            physical_position = add(
                physical_position,
                scale(direction(physical_heading), physical_distance),
            )

            if target_distance <= scenario.walking_speed * config.dt + 1e-9:
                virtual_position = target
                waypoint_index += 1
                if waypoint_index == len(scenario.virtual_waypoints):
                    if not scenario.loop_path:
                        break
                    waypoint_index = 0

            boundary_distance = scenario.space.signed_distance_to_boundary(physical_position)
            in_danger = boundary_distance < scenario.space.safety_margin
            reset = in_danger and not was_in_danger
            if reset:
                physical_heading = angle_of(
                    subtract(scenario.space.center, physical_position)
                )
            was_in_danger = in_danger

            records.append(
                StepRecord(
                    time=(step + 1) * config.dt,
                    physical_position=physical_position,
                    virtual_position=virtual_position,
                    boundary_distance=boundary_distance,
                    command=command,
                    reset=reset,
                    outside=boundary_distance < 0.0,
                    physical_distance=physical_distance,
                    virtual_distance=virtual_distance,
                    controller_time_ms=controller_time_ms,
                )
            )

        if not records:
            raise RuntimeError("Simulation produced no records.")
        return SimulationResult(scenario.name, controller.name, scenario.space, tuple(records))
