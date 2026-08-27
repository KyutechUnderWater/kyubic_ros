"""練習問題：PID計算と方位誤差の正規化を完成させる。"""


def clamp(value, limit):
    """valueを[-abs(limit), abs(limit)]へ制限する安全処理。"""
    limit = abs(float(limit))
    return max(-limit, min(float(value), limit))


def normalize_angle_error(target_deg, current_deg):
    """TODO 1：最短の方位誤差を-180以上180未満で返す。"""
    # ヒント：(target - current + 180)を360で割った余りを利用する。
    raise NotImplementedError("TODO 1: normalize_angle_error")


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
    """PIDを1周期分計算し、(出力、今回誤差、新しい積分値)を返す。"""
    if dt <= 0.0:
        raise ValueError("dt must be greater than zero")

    error = float(error)
    previous_integral = float(integral)

    # TODO 2：今回の誤差 error * dt を積分値へ加える。
    # clamp(..., integral_limit)を使って積分値を制限すること。
    candidate_integral = 0.0

    # TODO 3：P項を計算する。
    p_term = 0.0

    # TODO 4：I項を計算する。
    i_term = 0.0

    # TODO 5：D項を計算する。
    # 初回（has_previous_error == False）はderivativeを0.0にする。
    derivative = 0.0
    d_term = 0.0

    # TODO 6：P項、I項、D項を合計する。
    raw_output = 0.0

    # ここからは安全のため完成済み：飽和時の積分増加を取り消す。
    if abs(raw_output) > abs(output_limit) and raw_output * error > 0.0:
        candidate_integral = previous_integral
        i_term = ki * candidate_integral
        raw_output = p_term + i_term + d_term

    output = clamp(raw_output, output_limit)

    # TODO 7：output、今回のerror、candidate_integralを返す。
    raise NotImplementedError("TODO 7: return PID values")
