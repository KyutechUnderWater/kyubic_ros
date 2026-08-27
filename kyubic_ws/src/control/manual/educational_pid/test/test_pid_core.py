import math

import pytest

from educational_pid.pid_core import calculate_pid, clamp, normalize_angle_error


def run_pid(**overrides):
    values = {
        "error": 2.0,
        "kp": 3.0,
        "ki": 0.0,
        "kd": 0.0,
        "prev_error": 0.0,
        "integral": 0.0,
        "dt": 0.1,
        "integral_limit": 10.0,
        "output_limit": 100.0,
        "has_previous_error": False,
    }
    values.update(overrides)
    return calculate_pid(**values)


def test_clamp():
    assert clamp(12.0, 5.0) == 5.0
    assert clamp(-12.0, 5.0) == -5.0
    assert clamp(3.0, 5.0) == 3.0


def test_shortest_angle_error():
    assert normalize_angle_error(-179.0, 179.0) == pytest.approx(2.0)
    assert normalize_angle_error(179.0, -179.0) == pytest.approx(-2.0)
    assert normalize_angle_error(90.0, 0.0) == pytest.approx(90.0)


def test_proportional_term():
    output, error, integral = run_pid()
    assert output == pytest.approx(6.0)
    assert error == pytest.approx(2.0)
    assert integral == pytest.approx(0.2)


def test_integral_accumulates():
    output, _, integral = run_pid(
        kp=0.0,
        ki=2.0,
        integral=0.5,
        dt=0.25,
    )
    assert integral == pytest.approx(1.0)
    assert output == pytest.approx(2.0)


def test_first_derivative_is_zero():
    output, _, _ = run_pid(kp=0.0, kd=1.0, has_previous_error=False)
    assert output == pytest.approx(0.0)


def test_derivative_after_first_update():
    output, _, _ = run_pid(
        error=3.0,
        kp=0.0,
        kd=2.0,
        prev_error=2.0,
        dt=0.5,
        has_previous_error=True,
    )
    assert output == pytest.approx(4.0)


def test_output_and_integral_are_limited():
    output, _, integral = run_pid(
        error=100.0,
        kp=10.0,
        ki=1.0,
        integral=0.0,
        integral_limit=1.0,
        output_limit=5.0,
    )
    assert math.isfinite(output)
    assert output == pytest.approx(5.0)
    assert integral == pytest.approx(0.0)
