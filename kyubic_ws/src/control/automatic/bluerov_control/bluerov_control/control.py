"""位置/姿勢の外側PIDと速度/角速度の内側PIDからなる2段カスケード制御。

blueRovControl/controlfunction.py の EmergencyProblem 〜 control_tick を移植したもの。
PIDの計算式・カスケード構成自体は変更していないが、以下の点はROS2移植に伴い変わっている:

  - 姿勢/速度/深度の入力元が自前QEKF(BNO08x IMU + DVL自前パース)から
    /localization/odom の購読値に変わった(旧controlfunction.get_yaw_rate_degが
    行っていたQEKF推定ジャイロバイアス補正は、/localization/odomが生ジャイロ値ベース
    のため今回は行っていない。README「既知の制約」参照)。
  - 出力先がMAVLink MANUAL_CONTROL(-1000〜1000、500中立のPWM相当スケール)から
    geometry_msgs/WrenchStamped(mavlink_driverのaxis.<name>.limitパラメータで
    物理量として解釈されるスケール)に変わった。両者はスケールが異なるため、
    旧params.pyのPIDゲイン・output_limitsはそのまま数値流用できず、実機での
    リチューニングが必須(README「PIDゲインのチューニング」参照)。
"""

import math
from dataclasses import dataclass

from .pid import PID, VelocityPID
from .utility import rotation_matrix_from_euler_deg


@dataclass
class ControlResult:
    emergency: bool
    reached: bool
    force_x: float = 0.0  # WrenchStamped.wrench.force.x (forward) に渡す値
    force_z: float = 0.0  # WrenchStamped.wrench.force.z (heave) に渡す値
    torque_z: float = 0.0  # WrenchStamped.wrench.torque.z (yaw) に渡す値


def world_velocity_to_body_surge(
    vx_world: float,
    vy_world: float,
    vz_world: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> float:
    """world(NED)速度ベクトルを機体座標系に回転させ、前後(surge)成分だけを返す。"""
    r = rotation_matrix_from_euler_deg(roll_deg, pitch_deg, yaw_deg)  # body -> world
    v_body = r.T @ [vx_world, vy_world, vz_world]
    return float(v_body[0])


class ControlLoop:
    """位置制御(外側)と速度制御(内側)のカスケードPIDをまとめて保持・実行するクラス。"""

    def __init__(
        self,
        gains: dict,
        position_tolerance: float,
        depth_tolerance: float,
        surface_z_threshold: float,
    ):
        # 外側(位置)ループはKyubicのp_pid_controllerと同じく単純なP(clamp付き)なので
        # `PID`(位置形)をki=kd=0で使う。内側(速度)ループはKyubicの
        # pid_controller::VelocityPID(速度形PID)を移植した`VelocityPID`を使う。
        self._surge_outer_pid = PID(**gains["surge_outer"])
        self._heave_outer_pid = PID(**gains["heave_outer"])
        self._yaw_outer_pid = PID(**gains["yaw_outer"])
        self._surge_inner_pid = VelocityPID(**gains["surge_inner"])
        self._heave_inner_pid = VelocityPID(**gains["heave_inner"])
        self._yaw_inner_pid = VelocityPID(**gains["yaw_inner"])
        self._position_tolerance = position_tolerance
        self._depth_tolerance = depth_tolerance
        self._surface_z_threshold = surface_z_threshold
        self._was_holding = False

    def reset_all_pids(self) -> None:
        """目標到達後や緊急停止後など、PIDの内部状態をクリアしたい時に呼ぶ"""
        for pid in (
            self._surge_outer_pid,
            self._heave_outer_pid,
            self._yaw_outer_pid,
            self._surge_inner_pid,
            self._heave_inner_pid,
            self._yaw_inner_pid,
        ):
            pid.reset()

    def is_emergency(self, position_z: float, odom_stale: bool = False) -> bool:
        """position_z(NED、正=下方向)が浅すぎる(水面に近すぎる)場合、または
        odom_staleがTrue(/localization/odomが一定時間途絶えている。node.py側で判定)の場合にTrue。

        masudanote.md Day4「位置センサー(DVL・オドメトリ)の死亡監視」に基づく。
        odomが死んでいるのに位置制御を続けると計算が狂って暴走しうるため、position_z自体も
        古い値かもしれない前提でodom_staleを独立した条件として扱う(どちらか一方で緊急)。
        """
        return position_z < self._surface_z_threshold or odom_stale

    def is_target_reached(self, dx: float, dy: float, dz: float) -> bool:
        horizontal_distance = math.sqrt(dx**2 + dy**2)
        return horizontal_distance < self._position_tolerance and abs(dz) < self._depth_tolerance

    def tick(
        self,
        dx: float,
        dy: float,
        dz: float,
        position_z: float,
        yaw_deg: float,
        surge_velocity_body: float,
        heave_velocity_world: float,
        yaw_rate_deg: float,
        dt: float,
        odom_stale: bool = False,
    ) -> ControlResult:
        """制御ループの1周期分を実行する。

        Args:
            dx, dy, dz: 現在位置から目標までのオフセット [m] (NED)。FSM側が管理するtarget。
            position_z: 現在の深度 [m] (NED、正=下方向)。EmergencyProblem判定に使う。
            yaw_deg: 現在のyaw角 [deg]。
            surge_velocity_body: 機体座標系での前後方向速度 [m/s]。
            heave_velocity_world: world(NED)座標系での上下方向速度 [m/s]。
            yaw_rate_deg: 現在のyaw角速度 [deg/s]。
            dt: 前回tickからの経過時間 [s]。
            odom_stale: /localization/odomの最終受信からodom_timeout_sを超えた場合True
                (node.py側で判定して渡す)。

        Returns:
            ControlResult: emergency/reached状態と、WrenchStampedへ渡す各軸コマンド。
        """
        if self.is_emergency(position_z, odom_stale):
            return ControlResult(emergency=True, reached=False)

        if self.is_target_reached(dx, dy, dz):
            if not self._was_holding:
                self.reset_all_pids()  # ホールドに入る瞬間に一度だけリセット
                self._was_holding = True
            return ControlResult(emergency=False, reached=True)
        self._was_holding = False

        horizontal_distance = math.sqrt(dx**2 + dy**2)
        surge_setpoint = self._surge_outer_pid.update_from_error(horizontal_distance, dt)
        heave_setpoint = self._heave_outer_pid.update_from_error(dz, dt)
        target_azimuth_deg = math.degrees(math.atan2(dy, dx))
        yaw_rate_setpoint = self._yaw_outer_pid.update(target_azimuth_deg, yaw_deg, dt)

        x_cmd = self._surge_inner_pid.update(surge_setpoint, surge_velocity_body, dt)
        z_cmd = self._heave_inner_pid.update(heave_setpoint, heave_velocity_world, dt)
        r_cmd = self._yaw_inner_pid.update(yaw_rate_setpoint, yaw_rate_deg, dt)

        return ControlResult(
            emergency=False, reached=False, force_x=x_cmd, force_z=z_cmd, torque_z=r_cmd
        )
