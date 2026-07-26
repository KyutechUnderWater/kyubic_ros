"""Pure conversion helpers used by the BlueROV MAVLink driver."""

from __future__ import annotations

import math


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Return *value* constrained to the inclusive range."""
    return max(minimum, min(maximum, value))


def axis_to_pwm(
    value: float,
    limit: float,
    invert: bool,
    pwm_min: int,
    pwm_neutral: int,
    pwm_max: int,
) -> int:
    """Convert a signed physical command to an asymmetric RC PWM value."""
    if limit <= 0.0:
        raise ValueError("axis limit must be positive")

    normalized = clamp(value / limit, -1.0, 1.0)
    if invert:
        normalized = -normalized

    if normalized >= 0.0:
        pwm = pwm_neutral + normalized * (pwm_max - pwm_neutral)
    else:
        pwm = pwm_neutral + normalized * (pwm_neutral - pwm_min)
    return int(round(clamp(pwm, pwm_min, pwm_max)))


def angle_to_pwm(
    angle_deg: float,
    angle_min_deg: float,
    angle_max_deg: float,
    pwm_min: int,
    pwm_max: int,
) -> int:
    """Convert a camera angle in degrees to a PWM command."""
    if angle_max_deg <= angle_min_deg:
        raise ValueError("camera angle range must be positive")

    ratio = clamp((angle_deg - angle_min_deg) / (angle_max_deg - angle_min_deg), 0.0, 1.0)
    return int(round(pwm_min + ratio * (pwm_max - pwm_min)))


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Return an (x, y, z, w) quaternion from roll, pitch and yaw in radians."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def radians_to_degrees(value: float) -> float:
    """Convert an angular value in radians to degrees."""
    return math.degrees(value)
