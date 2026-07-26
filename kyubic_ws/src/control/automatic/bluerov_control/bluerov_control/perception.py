"""ハイドロフォンと画像処理(YOLO等)からのセンサ入力を扱う幾何計算モジュール。

blueRovControl/perception.py からの移植。get_latest_hydrophone_bearing() /
get_latest_yolo_detection() (共に NotImplementedError のプレースホルダだった)は、
ROS2化に伴い bluerov_control_msgs/PingerDirection・BuoyDetection トピックの
subscriber(node.py側)に置き換わったため、ここには残していない。
音響(SBL)側の「機体座標系の相対値→NED絶対座標」変換は sbl_controller_node が
planner_msgs/WrenchPlanとして計算済みの絶対座標を渡してくるため、このモジュールでは
行わない(bearing_to_body_offsetは削除済み)。本モジュールが持つのは画像処理
(BuoyDetection、機体座標系の相対値)をNED絶対座標に変換する純粋な幾何計算と、
各センサ入力の有効性判定のみ。
"""

import numpy as np

from .utility import rotation_matrix_from_euler_deg


def is_buoy_detection_usable(
    detection, now_sec: float, confidence_threshold: float, stale_timeout_sec: float
) -> bool:
    """confidenceが閾値未満、またはtimestampが古すぎる検出は使わない。

    detection: bluerov_control_msgs/BuoyDetection、または None(まだ一度も受信していない)。
    """
    if detection is None or not detection.detected:
        return False
    if detection.confidence < confidence_threshold:
        return False
    stamp_sec = detection.header.stamp.sec + detection.header.stamp.nanosec * 1e-9
    return not now_sec - stamp_sec > stale_timeout_sec


def is_pinger_direction_usable(pinger_direction, score_threshold: float) -> bool:
    """detectedがFalse、またはscoreが閾値未満の推定は使わない。

    pinger_direction: bluerov_control_msgs/PingerDirection、または None(まだ一度も受信していない)。
    """
    if pinger_direction is None or not pinger_direction.detected:
        return False
    return pinger_direction.score >= score_threshold


def body_offset_to_ned(
    position, roll_deg: float, pitch_deg: float, yaw_deg: float, body_offset
) -> np.ndarray:
    """
    機体座標系(body frame)の相対オフセットを、NED絶対座標に変換する。

    重要(要確認): relative_buoy_z_m / ハイドロフォンpitchの符号の向きが、
    このプロジェクト全体の慣習(NED、z正=下方向)と一致しているか、
    画像処理担当者・ハイドロフォンの仕様と必ずすり合わせること。
    ここがズレていると、ROVが目標と逆方向(上下逆・左右逆)に動く。
    """
    R = rotation_matrix_from_euler_deg(roll_deg, pitch_deg, yaw_deg)  # body -> world
    return np.asarray(position) + R @ np.array(body_offset)
