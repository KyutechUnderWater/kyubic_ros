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


class VelocityPID:
    """速度形(インクリメンタル)PID。

    common/pid_controller の pid_controller::VelocityPID(C++、Kyubicのwrench_plannerが
    実機で使っているカスケードPIDの内側ループ)と同じ計算式に移植したもの。`PID`(位置形)との違い:

    - 誤差そのものではなく前回誤差からの差分(p, d)を使い、出力は前回出力からの増分として
      計算したうえで毎周期 output_limits にクランプする。次周期はそのクランプ後の値から
      継続するため、`PID` のような専用の windup_limit が無くても積分ワインドアップが起きにくい。
    - D項に一次ローパスフィルタ(kf)をかけられる(kf=0 でフィルタなし、1に近いほど強くかかる)。
    - クランプ後に offset(浮力トリムなどのフィードフォワード項)を加算できる。
    """

    def __init__(self, kp, ki, kd, output_limits, kf=0.0, offset=0.0, angle_error_deg=False):
        """
        kp, ki, kd      : 各ゲイン
        output_limits   : (lower, upper) 出力のクランプ範囲(毎周期・offset加算前後の両方に適用)
        kf              : D項ローパスフィルタ係数。0でフィルタなし、1に近いほど強い平滑化
        offset          : クランプ後に加算するフィードフォワード項(例: 浮力トリム)
        angle_error_deg : True の場合、誤差を -180〜180 度に正規化する(yaw角そのものを扱う場合用)
        """
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limits = output_limits
        self.kf = kf
        self.offset = offset
        self.angle_error_deg = angle_error_deg
        self.reset()

    def reset(self):
        """内部状態をクリアする(制御対象を切り替えた時などに呼ぶ)"""
        self.pre_error = 0.0
        self.pre_p = 0.0
        self.pre_d = 0.0
        self.pre_u = 0.0
        self.initialized = False

    def set_offset(self, offset):
        self.offset = offset

    def _normalize(self, error):
        if self.angle_error_deg:
            return (error + 180.0) % 360.0 - 180.0
        return error

    def update(self, setpoint, measurement, dt):
        lo, hi = self.output_limits
        error = self._normalize(setpoint - measurement)

        if not self.initialized:
            # 初回はP項のみで出力を作る(前回出力が無いため増分を計算できない)
            self.pre_error = error
            self.pre_p = 0.0
            self.pre_d = 0.0
            self.pre_u = clamp(self.kp * error, lo, hi)
            self.initialized = True
            return clamp(self.pre_u + self.offset, lo, hi)

        if dt <= 0.0:
            # dtが取れない場合は前回出力を維持(急変を避ける)
            return clamp(self.pre_u + self.offset, lo, hi)

        p = error - self.pre_error
        i = error * dt
        d_raw = (p - self.pre_p) / dt
        d = self.kf * self.pre_d + (1.0 - self.kf) * d_raw

        u = clamp(self.pre_u + self.kp * p + self.ki * i + self.kd * d, lo, hi)

        self.pre_error = error
        self.pre_p = p
        self.pre_d = d
        self.pre_u = u
        return clamp(u + self.offset, lo, hi)
