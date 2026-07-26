import math
import os
import sys

_CONTROL_DIR = os.path.dirname(os.path.abspath(__file__))
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

from dataclasses import dataclass

import numpy as np

import utility_functions
from function import state_estimation, get_last_dt, get_last_imu_data
from mavlink_io import ManualControl
from params import (
    POSITION_TOLERANCE, DEPTH_TOLERANCE,
    PID_SURGE_OUTER, PID_HEAVE_OUTER, PID_YAW_OUTER,
    PID_SURGE_INNER, PID_HEAVE_INNER, PID_YAW_INNER,
)
from pid import PID


_surge_outer_pid = PID(**PID_SURGE_OUTER)
_heave_outer_pid = PID(**PID_HEAVE_OUTER)
_yaw_outer_pid = PID(**PID_YAW_OUTER)
_surge_inner_pid = PID(**PID_SURGE_INNER)
_heave_inner_pid = PID(**PID_HEAVE_INNER)
_yaw_inner_pid = PID(**PID_YAW_INNER)


# ============================================================
# 以下、EmergencyProblem 〜 reset_all_pids までは version0 から無改造。
# PIDの計算式・座標変換ロジックには一切手を触れていない。
# ============================================================

def EmergencyProblem(current_state):
    """
    current_state: QEKFの公称状態(19要素のnp.ndarray, function.state_estimation()の戻り値)。
    現在は current_state[2] (position z [m], NED, 正=下方向) のみで判定する。
    """
    if current_state[2] < 0.1:
        return True
    return False


def AzimuthControl(target_x, target_y):
    """目標座標 (target_x, target_y) [m, NED] へのyaw角を返す（度）"""
    target_azimuth_yaw_degree = math.degrees(math.atan2(target_y, target_x))
    return target_azimuth_yaw_degree


def OuterLoopControl(dx, dy, dz, current_yaw_deg, dt):
    """位置/yaw誤差から速度指令(setpoint)を計算する外側ループ。"""
    horizontal_distance = math.sqrt(dx**2 + dy**2)
    surge_setpoint = _surge_outer_pid.update_from_error(horizontal_distance, dt)
    heave_setpoint = _heave_outer_pid.update_from_error(dz, dt)
    target_azimuth_deg = AzimuthControl(dx, dy)
    yaw_rate_setpoint = _yaw_outer_pid.update(target_azimuth_deg, current_yaw_deg, dt)
    return surge_setpoint, heave_setpoint, yaw_rate_setpoint, target_azimuth_deg


def is_target_reached(target_x, target_y, target_z):
    """水平距離がPOSITION_TOLERANCE未満、かつ|target_z|がDEPTH_TOLERANCE未満で到達とみなす。"""
    return math.sqrt(target_x**2 + target_y**2) < POSITION_TOLERANCE and abs(target_z) < DEPTH_TOLERANCE


def get_body_frame_velocity(current_state):
    """QEKFのworld(NED)速度を機体座標系に回転させる。"""
    q = current_state[6:10]
    v_world = current_state[3:6]
    R = utility_functions.quaternion_to_rotation_matrix(q)
    return R.T @ v_world


def get_yaw_rate_deg(current_state):
    """生ジャイロ - QEKF推定バイアス から yawレート[deg/s]を求める。"""
    imu_data = get_last_imu_data()
    if imu_data is None:
        return 0.0
    gyro_z_raw = imu_data[1][2]
    gyro_bias_z = current_state[15]
    return math.degrees(gyro_z_raw - gyro_bias_z)


def InnerLoopControl(surge_setpoint, heave_setpoint, yaw_rate_setpoint, current_state, dt):
    """速度指令(setpoint)と実測値の誤差から、推力/トルク指令を計算する内側ループ。"""
    v_body = get_body_frame_velocity(current_state)
    surge_measured = v_body[0]
    heave_measured = current_state[5]
    yaw_rate_measured = get_yaw_rate_deg(current_state)

    x_cmd = _surge_inner_pid.update(surge_setpoint, surge_measured, dt)
    z_cmd_offset = _heave_inner_pid.update(heave_setpoint, heave_measured, dt)
    r_cmd = _yaw_inner_pid.update(yaw_rate_setpoint, yaw_rate_measured, dt)

    return x_cmd, z_cmd_offset, r_cmd


def reset_all_pids():
    """目標到達後や緊急停止後など、PIDの内部状態をクリアしたい時に呼ぶ"""
    _surge_outer_pid.reset()
    _heave_outer_pid.reset()
    _yaw_outer_pid.reset()
    _surge_inner_pid.reset()
    _heave_inner_pid.reset()
    _yaw_inner_pid.reset()


# ============================================================
# ここから Step1 の本題。run_control_loop() を置き換える新しい関数。
# ============================================================

@dataclass
class ControlResult:
    emergency: bool
    reached: bool
    position: "np.ndarray"    # 現在位置(NED)
    quaternion: "np.ndarray"  # 現在姿勢[qw,qx,qy,qz]。機体座標系→world座標系の回転変換に必要


_was_holding = False  # 元は run_control_loop() 内のローカル変数だった


def control_tick(dx, dy, dz):
    """
    制御ループの1周期分だけを実行する。

    元の run_control_loop() は while Flag: で自己完結した無限ループだったため、
    外側(FSM)が毎tick target を差し替えることができなかった。
    この関数はその中身を1回分だけ切り出し、target を引数で受け取るようにしたもの。
    PIDの計算式・座標変換ロジック自体は一切変更していない。

    現在位置(position)を戻り値に含めているのは、FSM側が「固定の目標地点に向かう」
    Stateを作る際、次tickのdx,dy,dzを計算するのに必要なため。state_estimation()を
    FSM側で呼び直すとQEKFがIMUデータを二重積分してしまうので、必ずこの戻り値を使う。

    Args:
        dx, dy, dz: 現在位置からの目標オフセット [m] (NED)。FSM側が管理するtarget。

    Returns:
        ControlResult(emergency, reached, position, quaternion)
    """
    global _was_holding

    current_state = state_estimation()
    position = current_state[0:3].copy()
    quaternion = current_state[6:10].copy()

    if EmergencyProblem(current_state):
        return ControlResult(emergency=True, reached=False, position=position, quaternion=quaternion)

    if is_target_reached(dx, dy, dz):
        if not _was_holding:
            reset_all_pids()   # ホールドに入る瞬間に一度だけリセット
            _was_holding = True
        ManualControl(0, 0, 500, 0)
        return ControlResult(emergency=False, reached=True, position=position, quaternion=quaternion)
    _was_holding = False

    dt = get_last_dt()
    _, _, current_yaw_rad = utility_functions.quaternion_to_euler(current_state[6:10])
    current_yaw_deg = math.degrees(current_yaw_rad)

    surge_sp, heave_sp, yaw_rate_sp, _ = OuterLoopControl(dx, dy, dz, current_yaw_deg, dt)
    x_cmd, z_cmd_offset, r_cmd = InnerLoopControl(surge_sp, heave_sp, yaw_rate_sp, current_state, dt)

    # heave_setpoint/heave_measuredはNED(正=沈む方向)、ManualControlのzは500中立・
    # 値が小さいほど沈む方向という逆の慣習のため、ここで符号を反転する。
    ManualControl(x_cmd, 0, -z_cmd_offset + 500, r_cmd)
    return ControlResult(emergency=False, reached=False, position=position, quaternion=quaternion)
