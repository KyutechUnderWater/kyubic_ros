"""PID制御器。

blueRovControl/pid.py からの移植。ROS2依存なし(dtは呼び出し側から渡される)。
"""

from .common import clamp


class PID:
    def __init__(self, kp, ki, kd, output_limits=None, windup_limit=None, angle_error_deg=False):
        """
        kp, ki, kd      : 各ゲイン
        output_limits   : (lower, upper) 出力のクランプ範囲。None ならクランプなし
        windup_limit    : 積分項(internal integral)自体のクランプ値。None ならクランプなし
                           (アンチワインドアップ。出力クランプとは別に必要)
        angle_error_deg : True の場合、誤差を -180〜180 度に正規化する(yaw等の
                           角度誤差用)。setpoint/measurement は度単位で渡すこと。
        """
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limits = output_limits
        self.windup_limit = windup_limit
        self.angle_error_deg = angle_error_deg
        self.reset()

    def reset(self):
        """積分項・微分用の前回誤差をクリアする(制御対象を切り替えた時などに呼ぶ)"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_output = 0.0
        self.initialized = False

    def _normalize(self, error):
        if self.angle_error_deg:
            return (error + 180.0) % 360.0 - 180.0
        return error

    def update(self, setpoint, measurement, dt):
        """setpoint, measurement から誤差を計算して更新する(速度PID向け)"""
        error = self._normalize(setpoint - measurement)
        return self._update_from_error(error, dt)

    def update_from_error(self, error, dt):
        """誤差が既に計算済みの場合に使う(位置PIDのように上流でerror算出済みのケース)"""
        return self._update_from_error(self._normalize(error), dt)

    def _update_from_error(self, error, dt):
        if dt <= 0.0:
            # 初回呼び出し等、dtが取れない場合は前回出力を維持(急変を避ける)
            return self.last_output if self.initialized else 0.0

        p_term = self.kp * error

        self.integral += error * dt
        if self.windup_limit is not None:
            self.integral = clamp(self.integral, -self.windup_limit, self.windup_limit)
        i_term = self.ki * self.integral

        d_term = 0.0
        if self.initialized:
            d_term = self.kd * (error - self.last_error) / dt

        output = p_term + i_term + d_term
        if self.output_limits is not None:
            lo, hi = self.output_limits
            output = clamp(output, lo, hi)

        self.last_error = error
        self.last_output = output
        self.initialized = True
        return output
