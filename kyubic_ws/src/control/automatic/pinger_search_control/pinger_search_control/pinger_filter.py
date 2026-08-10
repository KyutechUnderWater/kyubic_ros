"""ピンガー方向(/pinger_direction)の外れ値対策フィルタ。

ピンガーの推定方位はたまに飛ぶ(単発の外れ値が出る)ため、
  1. score(最適化誤差。小さいほど良い)が閾値を超えたサンプルを棄却
  2. 直近windowサンプルのメディアンを出力(単発スパイクを除去。yawは円環メディアン)
の2段で平滑化する。メディアンなので、ロボットが旋回して相対方位が正しく
変化していく分には1サンプル程度の遅れで追従する。
"""

import math
from collections import deque


def wrap_deg(angle: float) -> float:
    """角度を[-180, 180)に正規化する。"""
    return (angle + 180.0) % 360.0 - 180.0


def _circular_mean_deg(angles: list[float]) -> float:
    """円環角度[deg]の平均(ベクトル平均)。"""
    return math.degrees(
        math.atan2(
            sum(math.sin(math.radians(a)) for a in angles),
            sum(math.cos(math.radians(a)) for a in angles),
        )
    )


def _circular_median_deg(angles: list[float]) -> float:
    """円環角度[deg]のメディアン。円環平均まわりにunwrapしてから通常のメディアンを取る。"""
    mean = _circular_mean_deg(angles)
    unwrapped = sorted(mean + wrap_deg(a - mean) for a in angles)
    n = len(unwrapped)
    if n % 2 == 1:
        median = unwrapped[n // 2]
    else:
        median = 0.5 * (unwrapped[n // 2 - 1] + unwrapped[n // 2])
    return wrap_deg(median)


def robust_circular_mean_deg(angles: list[float], outlier_deg: float) -> float:
    """外れ値を除いた円環角度[deg]の平均。

    メディアンから outlier_deg より離れたサンプルを捨て、残りのベクトル平均を返す。
    全部捨ててしまった場合はメディアンを返す。step3/音響追尾の「数回観測して
    正面を合わせる」判定に使う。
    """
    median = _circular_median_deg(angles)
    kept = [a for a in angles if abs(wrap_deg(a - median)) <= outlier_deg]
    if not kept:
        return median
    return wrap_deg(_circular_mean_deg(kept))


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    n = len(ordered)
    if n % 2 == 1:
        return ordered[n // 2]
    return 0.5 * (ordered[n // 2 - 1] + ordered[n // 2])


class PingerFilter:
    def __init__(self, window: int = 3, score_max: float = 100.0):
        self._window = max(1, window)
        self._score_max = score_max
        self._yaw_buffer: deque[float] = deque(maxlen=self._window)
        self._pitch_buffer: deque[float] = deque(maxlen=self._window)

    def reset(self) -> None:
        self._yaw_buffer.clear()
        self._pitch_buffer.clear()

    def update(self, yaw_deg: float, pitch_deg: float, score: float) -> tuple[float, float] | None:
        """1サンプル追加し、採用ならフィルタ後の(yaw, pitch)[deg]を返す。棄却ならNone。"""
        if not (math.isfinite(yaw_deg) and math.isfinite(pitch_deg) and math.isfinite(score)):
            return None
        if score > self._score_max:
            return None
        self._yaw_buffer.append(wrap_deg(yaw_deg))
        self._pitch_buffer.append(pitch_deg)
        return (
            _circular_median_deg(list(self._yaw_buffer)),
            _median(list(self._pitch_buffer)),
        )
