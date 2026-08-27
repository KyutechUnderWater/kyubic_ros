"""Pure PID calculations used by the educational controller."""


def clamp(value, limit):
    """Clamp ``value`` to ``[-abs(limit), abs(limit)]``."""
    limit = abs(float(limit))
    return max(-limit, min(float(value), limit))


def normalize_angle_error(target_deg, current_deg):
    """Return the shortest signed angular error in degrees."""
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def calculate_pid(
    error,
    kp,
    ki,
    kd,
    prev_error,
    integral,
    dt,
    integral_limit,
    output_limit,
    has_previous_error,
):
    """Calculate one position-form PID update with simple anti-windup.

    Returns ``(output, current_error, new_integral)``. The derivative term
    is disabled on the first update to avoid a derivative kick at startup.
    """
    if dt <= 0.0:
        raise ValueError("dt must be greater than zero")

    error = float(error)
    previous_integral = float(integral)
    candidate_integral = clamp(
        previous_integral + error * dt,
        integral_limit,
    )

    p_term = kp * error
    i_term = ki * candidate_integral
    derivative = (error - prev_error) / dt if has_previous_error else 0.0
    d_term = kd * derivative
    raw_output = p_term + i_term + d_term

    # Saturation and the current error are pushing the output farther in
    # the same direction, so discard this cycle's integral increase.
    if abs(raw_output) > abs(output_limit) and raw_output * error > 0.0:
        candidate_integral = previous_integral
        i_term = ki * candidate_integral
        raw_output = p_term + i_term + d_term

    output = clamp(raw_output, output_limit)
    return output, error, candidate_integral
