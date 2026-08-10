"""深度・ヨー保持用のシンプルな位置形PID。

localization/DVLを使わないため、bluerov_controlのカスケードPID(位置外側+速度内側)
ではなく、センサ値(深度[m]・ヨー角[deg])に対する単段PIDで直接robot_forceを出す。
"""


class PID:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float,
        integral_limit: float = 0.0,
    ):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._output_limit = abs(output_limit)
        # integral_limit=0なら出力リミットと同値でクランプ(素朴なanti-windup)
        self._integral_limit = abs(integral_limit) if integral_limit else self._output_limit
        self._integral = 0.0
        self._previous_error: float | None = None

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_error = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            dt = 1e-3
        self._integral += error * dt
        if self._ki > 0.0:
            self._integral = max(
                -self._integral_limit / self._ki,
                min(self._integral_limit / self._ki, self._integral),
            )
        derivative = 0.0
        if self._previous_error is not None:
            derivative = (error - self._previous_error) / dt
        self._previous_error = error
        output = self._kp * error + self._ki * self._integral + self._kd * derivative
        return max(-self._output_limit, min(self._output_limit, output))
