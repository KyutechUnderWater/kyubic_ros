"""Pure safety calculations used by the Day 4 Wrench gate."""

import math


def clamp_symmetric(value, limit):
    """Clamp ``value`` to ``[-abs(limit), abs(limit)]``."""
    value = float(value)
    limit = abs(float(limit))
    return max(-limit, min(value, limit))


def values_are_finite(*values):
    """Return True only when every command value is a finite number."""
    return all(math.isfinite(float(value)) for value in values)


def sanitize_command(
    force_x,
    force_y,
    force_z,
    torque_z,
    *,
    allow_manual_xy,
    allow_depth,
    allow_yaw,
    max_force_x,
    max_force_y,
    max_force_z,
    max_torque_z,
):
    """Validate, select enabled axes, and clamp one Wrench command.

    The return order is ``force_x, force_y, force_z, torque_z``.
    ``ValueError`` is raised for NaN or infinity so the caller can disarm.
    """
    values = (force_x, force_y, force_z, torque_z)
    if not values_are_finite(*values):
        raise ValueError("Wrench command contains NaN or infinity")

    safe_force_x = clamp_symmetric(force_x, max_force_x) if allow_manual_xy else 0.0
    safe_force_y = clamp_symmetric(force_y, max_force_y) if allow_manual_xy else 0.0
    safe_force_z = clamp_symmetric(force_z, max_force_z) if allow_depth else 0.0
    safe_torque_z = clamp_symmetric(torque_z, max_torque_z) if allow_yaw else 0.0
    return safe_force_x, safe_force_y, safe_force_z, safe_torque_z
