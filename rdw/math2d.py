from __future__ import annotations

import math

Vec2 = tuple[float, float]


def add(a: Vec2, b: Vec2) -> Vec2:
    return a[0] + b[0], a[1] + b[1]


def subtract(a: Vec2, b: Vec2) -> Vec2:
    return a[0] - b[0], a[1] - b[1]


def scale(vector: Vec2, factor: float) -> Vec2:
    return vector[0] * factor, vector[1] * factor


def length(vector: Vec2) -> float:
    return math.hypot(vector[0], vector[1])


def direction(angle: float) -> Vec2:
    return math.cos(angle), math.sin(angle)


def angle_of(vector: Vec2) -> float:
    return math.atan2(vector[1], vector[0])


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
