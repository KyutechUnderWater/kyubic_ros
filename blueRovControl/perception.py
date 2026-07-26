"""
ハイドロフォンと画像処理(YOLO、別担当者が実装)からのセンサ入力を扱うモジュール。

どちらも「機体座標系(body frame)における相対値」を返す点が共通しているため、
NED絶対座標に変換する共通ヘルパー(body_offset_to_ned)もここに置く。

実際のハードウェア読み出し/プロセス間連携の方法はまだ決まっていないため、
get_latest_hydrophone_bearing() と get_latest_yolo_detection() はプレースホルダ。
安全側に倒すため、例外を投げずに「値なし」を表す戻り値を返す設計にしている
(呼び出し側のFSMがNoneチェック1つで済むように)。
"""
import math
from dataclasses import dataclass

import numpy as np

import utility_functions
from params import BINGER_SEARCH_ASSUMED_DISTANCE, YOLO_CONFIDENCE_THRESHOLD, YOLO_STALE_TIMEOUT


@dataclass
class HydrophoneBearing:
    pitch_deg: float  # 仰角/俯角。正 = 対象が下方向(body z正方向、NED慣習に合わせる)
    yaw_deg: float     # 水平面内の方位角。前方(body x正方向)を0度として反時計回りが正
    timestamp: float


@dataclass
class YoloDetection:
    detected: bool
    x_m: float          # 機体座標系: 前後方向(前方が正)
    y_m: float          # 機体座標系: 左右方向
    z_m: float          # 機体座標系: 上下方向
    confidence: float
    timestamp: float


def get_latest_hydrophone_bearing():
    """
    ハイドロフォンの最新の pitch, yaw を取得する。
    TODO: 実際のハードウェア読み出し(シリアル/ソケット等)が決まり次第置き換える。
    """
    raise NotImplementedError("ハイドロフォンの実読み出し方法が決まり次第実装する")


def get_latest_yolo_detection():
    """
    画像処理担当者のプロセスから最新の検出結果を取得する。
    TODO: 実際の受信方法(ROS2トピック購読 / 共有メモリ / ファイル等)が決まり次第置き換える。
    """
    raise NotImplementedError("画像処理側との実際の連携方法が決まり次第実装する")


def is_yolo_detection_usable(detection, now):
    """confidenceが閾値未満、またはtimestampが古すぎる検出は使わない。"""
    if detection is None or not detection.detected:
        return False
    if detection.confidence < YOLO_CONFIDENCE_THRESHOLD:
        return False
    if now - detection.timestamp > YOLO_STALE_TIMEOUT:
        return False
    return True


def bearing_to_body_offset(bearing, assumed_distance=BINGER_SEARCH_ASSUMED_DISTANCE):
    """
    ハイドロフォンの pitch, yaw [度] から、機体座標系における相対オフセット (bx, by, bz) [m] を返す。
    距離は測定できないため assumed_distance [m] を仮定する(球面座標→直交座標変換)。
    """
    pitch = math.radians(bearing.pitch_deg)
    yaw = math.radians(bearing.yaw_deg)
    horizontal = assumed_distance * math.cos(pitch)
    bx = horizontal * math.cos(yaw)
    by = horizontal * math.sin(yaw)
    bz = assumed_distance * math.sin(pitch)
    return (bx, by, bz)


def body_offset_to_ned(position, quaternion, body_offset):
    """
    機体座標系(body frame)の相対オフセットを、NED絶対座標に変換する。

    重要(要確認): relative_buoy_z_m / ハイドロフォンpitchの符号の向きが、
    このプロジェクト全体の慣習(NED、z正=下方向)と一致しているか、
    画像処理担当者・ハイドロフォンの仕様と必ずすり合わせること。
    ここがズレていると、ROVが目標と逆方向(上下逆・左右逆)に動く。
    """
    R = utility_functions.quaternion_to_rotation_matrix(quaternion)  # body -> world
    return np.asarray(position) + R @ np.array(body_offset)
