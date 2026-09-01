#!/usr/bin/env python3
from __future__ import annotations

import math


def step_towards(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


def shape_deadzone_target(target: float, min_effective: float, zero_epsilon: float) -> float:
    if abs(target) <= zero_epsilon:
        return 0.0
    if min_effective > 0.0 and abs(target) < min_effective:
        return math.copysign(min_effective, target)
    return target


def limit_deadzone_axis(
    current: float,
    target: float,
    dt: float,
    max_accel: float,
    max_decel: float,
    min_effective: float,
    zero_epsilon: float,
) -> float:
    dt = max(float(dt), 1.0e-3)
    max_accel = max(float(max_accel), 0.0)
    max_decel = max(float(max_decel), 0.0)
    min_effective = max(float(min_effective), 0.0)
    zero_epsilon = max(float(zero_epsilon), 0.0)
    shaped_target = shape_deadzone_target(target, min_effective, zero_epsilon)

    if min_effective <= 0.0:
        accelerating = current * shaped_target >= 0.0 and abs(shaped_target) > abs(current)
        limit = max_accel if accelerating else max_decel
        return step_towards(current, shaped_target, limit * dt)

    effective_current = 0.0 if abs(current) < min_effective else current
    if shaped_target == 0.0:
        stepped = step_towards(effective_current, 0.0, max_decel * dt)
        return 0.0 if abs(stepped) < min_effective else stepped

    if effective_current == 0.0:
        return math.copysign(min_effective, shaped_target)

    if effective_current * shaped_target < 0.0:
        stepped = step_towards(effective_current, 0.0, max_decel * dt)
        if stepped == 0.0 or abs(stepped) < min_effective:
            return 0.0
        return stepped

    accelerating = abs(shaped_target) > abs(effective_current)
    limit = max_accel if accelerating else max_decel
    stepped = step_towards(effective_current, shaped_target, limit * dt)
    if abs(stepped) < min_effective:
        return math.copysign(min_effective, shaped_target)
    return stepped
