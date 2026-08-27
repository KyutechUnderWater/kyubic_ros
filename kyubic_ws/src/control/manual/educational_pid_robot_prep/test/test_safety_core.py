import math

import pytest

from educational_pid_robot_prep.safety_core import (
    clamp_symmetric,
    sanitize_command,
    values_are_finite,
)


def test_clamp_symmetric():
    assert clamp_symmetric(3.0, 2.0) == 2.0
    assert clamp_symmetric(-3.0, 2.0) == -2.0
    assert clamp_symmetric(1.0, -2.0) == 1.0


def test_values_are_finite():
    assert values_are_finite(0.0, 1.0, -2.0)
    assert not values_are_finite(math.nan)
    assert not values_are_finite(math.inf)


def test_disabled_axes_are_zero_and_enabled_axes_are_clamped():
    command = sanitize_command(
        10.0,
        -10.0,
        4.0,
        -2.0,
        allow_manual_xy=False,
        allow_depth=True,
        allow_yaw=True,
        max_force_x=1.0,
        max_force_y=1.0,
        max_force_z=2.0,
        max_torque_z=0.5,
    )
    assert command == (0.0, 0.0, 2.0, -0.5)


def test_non_finite_command_is_rejected():
    with pytest.raises(ValueError):
        sanitize_command(
            0.0,
            0.0,
            math.nan,
            0.0,
            allow_manual_xy=False,
            allow_depth=True,
            allow_yaw=True,
            max_force_x=0.0,
            max_force_y=0.0,
            max_force_z=2.0,
            max_torque_z=0.5,
        )
