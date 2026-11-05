from __future__ import annotations

from dataclasses import dataclass, replace

from rdw.math2d import Vec2


@dataclass(frozen=True)
class AxisAlignedSpace:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    safety_margin: float = 0.35

    def __post_init__(self) -> None:
        if self.min_x >= self.max_x or self.min_y >= self.max_y:
            raise ValueError("Space bounds must have positive area.")
        if self.safety_margin < 0:
            raise ValueError("Safety margin cannot be negative.")

    @property
    def center(self) -> Vec2:
        return (self.min_x + self.max_x) / 2, (self.min_y + self.max_y) / 2

    def signed_distance_to_boundary(self, point: Vec2) -> float:
        """Positive inside the room and negative outside it."""
        x, y = point
        return min(x - self.min_x, self.max_x - x, y - self.min_y, self.max_y - y)


@dataclass(frozen=True)
class ControlLimits:
    max_abs_curvature: float = 0.25
    min_translation_gain: float = 0.86
    max_translation_gain: float = 1.26
    min_rotation_gain: float = 0.67
    max_rotation_gain: float = 1.24


@dataclass(frozen=True)
class Observation:
    step: int
    dt: float
    physical_position: Vec2
    physical_heading: float
    virtual_position: Vec2
    virtual_heading: float
    walking_speed: float
    space: AxisAlignedSpace
    # The simulator clamps every command to these limits; controllers read them
    # so that all algorithms share one set of perceptual thresholds.
    limits: ControlLimits = ControlLimits()


@dataclass(frozen=True)
class RedirectionCommand:
    curvature: float = 0.0
    translation_gain: float = 1.0
    rotation_gain: float = 1.0

    def clamped(self, limits: ControlLimits) -> "RedirectionCommand":
        return replace(
            self,
            curvature=max(-limits.max_abs_curvature, min(self.curvature, limits.max_abs_curvature)),
            translation_gain=max(
                limits.min_translation_gain,
                min(self.translation_gain, limits.max_translation_gain),
            ),
            rotation_gain=max(
                limits.min_rotation_gain,
                min(self.rotation_gain, limits.max_rotation_gain),
            ),
        )


@dataclass(frozen=True)
class Scenario:
    name: str
    space: AxisAlignedSpace
    virtual_waypoints: tuple[Vec2, ...]
    initial_physical_position: Vec2 = (0.0, 0.0)
    initial_physical_heading: float = 0.0
    walking_speed: float = 1.2
    loop_path: bool = True

    def __post_init__(self) -> None:
        if len(self.virtual_waypoints) < 2:
            raise ValueError("A scenario requires at least two virtual waypoints.")
        if self.walking_speed <= 0:
            raise ValueError("Walking speed must be positive.")


@dataclass(frozen=True)
class SimulationConfig:
    dt: float = 0.05
    duration_seconds: float = 60.0
    max_virtual_turn_rate: float = 1.5
    limits: ControlLimits = ControlLimits()


@dataclass(frozen=True)
class StepRecord:
    time: float
    physical_position: Vec2
    virtual_position: Vec2
    boundary_distance: float
    command: RedirectionCommand
    reset: bool
    outside: bool
    physical_distance: float
    virtual_distance: float
    controller_time_ms: float
