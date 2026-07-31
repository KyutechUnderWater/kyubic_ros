from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """歪み補正済み画像に対するカメラ内部パラメータ。"""

    fx: float
    fy: float
    cx: float
    cy: float
    calibration_width: int
    calibration_height: int

    def validate(self) -> None:
        values = (self.fx, self.fy, self.cx, self.cy)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("カメラ内部パラメータに有限でない値があります。")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError(
                "fxとfyにはカメラ校正で求めた正の値を設定してください。"
            )
        if self.calibration_width <= 0 or self.calibration_height <= 0:
            raise ValueError("校正画像の解像度が無効です。")

    def scaled_to(
        self,
        image_width: int,
        image_height: int,
    ) -> "CameraIntrinsics":
        """同一画角の解像度変更に合わせて内部パラメータを拡大縮小する。"""
        self.validate()

        if image_width <= 0 or image_height <= 0:
            raise ValueError("入力画像の解像度が無効です。")

        scale_x = image_width / self.calibration_width
        scale_y = image_height / self.calibration_height

        return CameraIntrinsics(
            fx=self.fx * scale_x,
            fy=self.fy * scale_y,
            cx=self.cx * scale_x,
            cy=self.cy * scale_y,
            calibration_width=image_width,
            calibration_height=image_height,
        )


@dataclass(frozen=True)
class BoundingBox:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def center_u(self) -> float:
        return (self.x1 + self.x2) * 0.5

    @property
    def center_v(self) -> float:
        return (self.y1 + self.y2) * 0.5

    def validate(self) -> None:
        values = (self.x1, self.y1, self.x2, self.y2)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("BBoxに有限でない値があります。")
        if self.width <= 1.0 or self.height <= 1.0:
            raise ValueError("BBoxの幅または高さが小さすぎます。")

    def touches_image_border(
        self,
        image_width: int,
        image_height: int,
        margin_px: int = 2,
    ) -> bool:
        self.validate()

        margin = float(max(margin_px, 0))
        return (
            self.x1 <= margin
            or self.y1 <= margin
            or self.x2 >= float(image_width - 1) - margin
            or self.y2 >= float(image_height - 1) - margin
        )


def calculate_normalized_horizontal_offset(
    bbox: BoundingBox,
    image_width: int,
) -> float:
    """BBox中心の符号付き水平偏差を画像半幅で正規化する。

    画像中心を0、右方向を正、左方向を負とする。
    BBox中心が画像内にある通常の条件では、おおむね-1～+1となる。
    この値は焦点距離、主点、対象距離に依存しない。
    """
    bbox.validate()

    if image_width <= 1:
        raise ValueError("画像幅は2 pixel以上である必要があります。")

    image_center_u = (float(image_width) - 1.0) * 0.5
    half_width = float(image_width) * 0.5
    offset = (bbox.center_u - image_center_u) / half_width

    if not math.isfinite(offset):
        raise ValueError("正規化水平偏差が有限値ではありません。")

    # BBoxの僅かな画像外は検出器の丸めで起こり得るため安全に制限する。
    return float(np.clip(offset, -1.0, 1.0))


def calculate_bbox_scale(
    bbox: BoundingBox,
    image_width: int,
    image_height: int,
) -> float:
    """BBox面積率の平方根を返す。"""
    bbox.validate()

    if image_width <= 0 or image_height <= 0:
        raise ValueError("画像の解像度が無効です。")

    return math.sqrt(
        (bbox.width * bbox.height)
        / float(image_width * image_height)
    )


def estimate_range_pinhole_area(
    bbox: BoundingBox,
    intrinsics: CameraIntrinsics,
    buoy_width_m: float,
    buoy_height_m: float,
) -> float:
    """既知物体寸法とBBox面積から光軸方向距離を概算する。"""
    bbox.validate()
    intrinsics.validate()

    if buoy_width_m <= 0.0 or buoy_height_m <= 0.0:
        raise ValueError("ブイの実寸は正の値で指定してください。")

    return math.sqrt(
        (
            intrinsics.fx
            * intrinsics.fy
            * buoy_width_m
            * buoy_height_m
        )
        / (bbox.width * bbox.height)
    )


def estimate_range_inverse_scale(
    bbox: BoundingBox,
    image_width: int,
    image_height: int,
    coefficient_a: float,
    coefficient_b: float,
) -> float:
    """実測校正式 Zc = a / scale + b から距離を推定する。"""
    if not math.isfinite(coefficient_a) or coefficient_a <= 0.0:
        raise ValueError("scale_aには校正済みの正の値を設定してください。")
    if not math.isfinite(coefficient_b):
        raise ValueError("scale_bには有限値を設定してください。")

    scale = calculate_bbox_scale(
        bbox=bbox,
        image_width=image_width,
        image_height=image_height,
    )
    return coefficient_a / scale + coefficient_b


def project_bbox_center_to_camera(
    bbox: BoundingBox,
    intrinsics: CameraIntrinsics,
    range_z_m: float,
) -> np.ndarray:
    """BBox中心をカメラ光学座標系へ逆投影する。

    camera optical frame:
      Xc: 画像右
      Yc: 画像下
      Zc: カメラ前方
    """
    bbox.validate()
    intrinsics.validate()

    if not math.isfinite(range_z_m) or range_z_m <= 0.0:
        raise ValueError("推定距離は正の有限値である必要があります。")

    x_camera_m = (
        (bbox.center_u - intrinsics.cx)
        * range_z_m
        / intrinsics.fx
    )
    y_camera_m = (
        (bbox.center_v - intrinsics.cy)
        * range_z_m
        / intrinsics.fy
    )

    return np.asarray(
        [x_camera_m, y_camera_m, range_z_m],
        dtype=np.float64,
    )




def estimate_body_yaw_from_bbox_center(
    bbox: BoundingBox,
    intrinsics: CameraIntrinsics,
    rotation_body_from_camera: Sequence[Sequence[float]],
) -> float:
    """BBox中心方向の機体座標系ヨー角を求める。

    カメラ光学座標の視線ベクトルを機体座標へ回転し、
    base_link FRDの水平面で atan2(Yb, Xb) を計算する。
    右方向を正、左方向を負とし、返り値の単位はrad。
    並進は方向角に影響しないため使用しない。
    """
    bbox.validate()
    intrinsics.validate()

    rotation = np.asarray(
        rotation_body_from_camera,
        dtype=np.float64,
    )
    if rotation.shape != (3, 3):
        raise ValueError("回転行列は3×3である必要があります。")
    if not np.all(np.isfinite(rotation)):
        raise ValueError("回転行列に有限でない値があります。")

    orthogonality_error = np.linalg.norm(
        rotation @ rotation.T - np.eye(3)
    )
    determinant = float(np.linalg.det(rotation))
    if orthogonality_error > 1.0e-5 or abs(determinant - 1.0) > 1.0e-5:
        raise ValueError(
            "camera_to_body_rotationは正しい回転行列ではありません。"
        )

    ray_camera = np.asarray(
        [
            (bbox.center_u - intrinsics.cx) / intrinsics.fx,
            (bbox.center_v - intrinsics.cy) / intrinsics.fy,
            1.0,
        ],
        dtype=np.float64,
    )
    ray_body = rotation @ ray_camera

    forward = float(ray_body[0])
    right = float(ray_body[1])
    if not math.isfinite(forward) or not math.isfinite(right):
        raise ValueError("ヨー角計算用の視線ベクトルが無効です。")
    if math.hypot(forward, right) <= 1.0e-12:
        raise ValueError("ヨー角を定義できない視線ベクトルです。")

    return math.atan2(right, forward)

def transform_camera_to_body(
    point_camera_xyz: Sequence[float],
    rotation_body_from_camera: Sequence[Sequence[float]],
    translation_body_from_camera_m: Sequence[float],
) -> np.ndarray:
    """カメラ光学座標を機体base_link座標へ変換する。

    P_body = R_body_camera @ P_camera + t_body_camera

    このパッケージで採用するbase_link:
      Xb: 機体前方
      Yb: 機体右方向
      Zb: 機体下方向
    """
    point = np.asarray(point_camera_xyz, dtype=np.float64)
    rotation = np.asarray(
        rotation_body_from_camera,
        dtype=np.float64,
    )
    translation = np.asarray(
        translation_body_from_camera_m,
        dtype=np.float64,
    )

    if point.shape != (3,):
        raise ValueError("カメラ座標は3要素である必要があります。")
    if rotation.shape != (3, 3):
        raise ValueError("回転行列は3×3である必要があります。")
    if translation.shape != (3,):
        raise ValueError("並進ベクトルは3要素である必要があります。")

    if not np.all(np.isfinite(point)):
        raise ValueError("カメラ座標に有限でない値があります。")
    if not np.all(np.isfinite(rotation)):
        raise ValueError("回転行列に有限でない値があります。")
    if not np.all(np.isfinite(translation)):
        raise ValueError("並進ベクトルに有限でない値があります。")

    orthogonality_error = np.linalg.norm(
        rotation @ rotation.T - np.eye(3)
    )
    determinant = float(np.linalg.det(rotation))
    if orthogonality_error > 1.0e-5 or abs(determinant - 1.0) > 1.0e-5:
        raise ValueError(
            "camera_to_body_rotationは正しい回転行列ではありません。"
        )

    return rotation @ point + translation
