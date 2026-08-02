from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
from buoy_interfaces.msg import BuoyRelativePosition
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO
import yaml

from buoy_detector.buoy_geometry import (
    BoundingBox,
    CameraIntrinsics,
    calculate_normalized_horizontal_offset,
    calculate_normalized_vertical_offset,
    estimate_body_yaw_from_bbox_center,
    estimate_range_inverse_scale,
    estimate_range_pinhole_area,
    project_bbox_center_to_camera,
    transform_camera_to_body,
)


# フレームごとに保存するCSVカラム。
CSV_FIELDS = [
    "frame_index",
    "timestamp_sec",
    "detected",
    "yaw_valid",
    "position_valid",
    "x",
    "y",
    "z",
    "yaw",
    "horizontal_offset",
    "vertical_offset",
    "confidence",
]

# ユーザー指定の固定保存先・固定ファイル名。
CSV_PATH = (
    Path.home()
    / "kyubic_ws"
    / "src"
    / "control"
    / "automatic"
    / "planning"
    / "path_planner"
    / "assets"
    / "iwakuni"
    / "bouy_point.csv"
)


def require_mapping(
    parent: dict[str, Any],
    key: str,
) -> dict[str, Any]:
    """YAML内の必須マッピングを取得する。"""
    value = parent.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"configの'{key}'がありません。")
    return value


def load_config(config_path: Path) -> dict[str, Any]:
    """ブイ位置・ヨー角推定用YAMLを読み込む。"""
    if not config_path.is_file():
        raise FileNotFoundError(
            f"座標計算configがありません: {config_path}"
        )

    with config_path.open("r", encoding="utf-8-sig") as file:
        config = yaml.safe_load(file)

    if not isinstance(config, dict):
        raise ValueError(
            "configのルートはマッピングである必要があります。"
        )

    require_mapping(config, "camera")
    require_mapping(config, "range")
    require_mapping(config, "validation")
    require_mapping(config, "transform")
    return config


def build_intrinsics(
    config: dict[str, Any],
    image_width: int,
    image_height: int,
) -> CameraIntrinsics:
    """入力画像解像度に合わせたカメラ内部パラメータを作る。"""
    camera = require_mapping(config, "camera")

    intrinsics = CameraIntrinsics(
        fx=float(camera["fx"]),
        fy=float(camera["fy"]),
        cx=float(camera["cx"]),
        cy=float(camera["cy"]),
        calibration_width=int(camera["calibration_width"]),
        calibration_height=int(camera["calibration_height"]),
    )

    return intrinsics.scaled_to(
        image_width=image_width,
        image_height=image_height,
    )


def calculate_body_yaw(
    bbox: BoundingBox,
    image_width: int,
    image_height: int,
    config: dict[str, Any],
) -> float:
    """BBox中心方向のbase_link FRDヨー角[rad]を計算する。"""
    intrinsics = build_intrinsics(
        config=config,
        image_width=image_width,
        image_height=image_height,
    )
    transform = require_mapping(config, "transform")
    return estimate_body_yaw_from_bbox_center(
        bbox=bbox,
        intrinsics=intrinsics,
        rotation_body_from_camera=transform[
            "camera_to_body_rotation"
        ],
    )


def calculate_body_position(
    bbox: BoundingBox,
    image_width: int,
    image_height: int,
    config: dict[str, Any],
) -> np.ndarray:
    """BBoxからbase_link（X前・Y右・Z下）のブイ相対位置を計算する。"""
    intrinsics = build_intrinsics(
        config=config,
        image_width=image_width,
        image_height=image_height,
    )

    range_config = require_mapping(config, "range")
    range_model = str(range_config["model"])

    min_range_m = float(range_config.get("min_range_m", 0.1))
    max_range_m = float(range_config.get("max_range_m", 50.0))

    if range_model == "pinhole_area":
        range_z_m = estimate_range_pinhole_area(
            bbox=bbox,
            intrinsics=intrinsics,
            buoy_width_m=float(range_config["buoy_width_m"]),
            buoy_height_m=float(range_config["buoy_height_m"]),
            min_range_m=min_range_m,
            max_range_m=max_range_m,
        )
    elif range_model == "inverse_scale":
        range_z_m = estimate_range_inverse_scale(
            bbox=bbox,
            image_width=image_width,
            image_height=image_height,
            coefficient_a=float(range_config["scale_a"]),
            coefficient_b=float(range_config["scale_b"]),
        )
    else:
        raise ValueError(
            "range.modelは'pinhole_area'または'inverse_scale'です。"
        )

    if not min_range_m <= range_z_m <= max_range_m:
        raise ValueError(
            f"推定距離 ({range_z_m:.2f} m) が許容範囲外 "
            f"[{min_range_m}, {max_range_m}] です。"
        )

    point_camera = project_bbox_center_to_camera(
        bbox=bbox,
        intrinsics=intrinsics,
        range_z_m=range_z_m,
    )

    transform = require_mapping(config, "transform")
    return transform_camera_to_body(
        point_camera_xyz=point_camera,
        rotation_body_from_camera=transform[
            "camera_to_body_rotation"
        ],
        translation_body_from_camera_m=transform[
            "camera_to_body_translation_m"
        ],
    )


class BuoyDetectorNode(Node):
    """ROS画像からブイを検出し、base_link座標とヨー角を配信する。"""

    def __init__(self) -> None:
        super().__init__("buoy_detector")

        package_share = Path(
            get_package_share_directory("buoy_detector")
        )

        self.declare_parameter(
            "model_path",
            str(package_share / "models" / "best.pt"),
        )
        self.declare_parameter(
            "config_path",
            str(package_share / "config" / "buoy_position.yaml"),
        )
        self.declare_parameter("input_image_topic", "/driver/blue_rov/camera/image_raw")
        self.declare_parameter(
            "output_topic",
            "/buoy/relative_position",
        )
        self.declare_parameter("body_frame_id", "base_link")
        self.declare_parameter("imgsz", 320)
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("class_id", 0)
        # 暫定内部パラメータで距離を出さないよう、安全側を既定値にする。
        self.declare_parameter("position_estimation_enabled", True)

        # ノード起動時に固定CSVを初期化し、画像受信ごとに1行追加する。
        self.declare_parameter("csv_logging_enabled", True)

        model_path = Path(
            str(self.get_parameter("model_path").value)
        ).expanduser()
        config_path = Path(
            str(self.get_parameter("config_path").value)
        ).expanduser()

        self._input_image_topic = str(
            self.get_parameter("input_image_topic").value
        )
        self._output_topic = str(
            self.get_parameter("output_topic").value
        )
        self._body_frame_id = str(
            self.get_parameter("body_frame_id").value
        )
        self._imgsz = int(self.get_parameter("imgsz").value)
        self._confidence_threshold = float(
            self.get_parameter("confidence").value
        )
        self._device = str(self.get_parameter("device").value)
        self._class_id = int(self.get_parameter("class_id").value)
        self._position_estimation_enabled = bool(
            self.get_parameter("position_estimation_enabled").value
        )
        self._csv_logging_enabled = bool(
            self.get_parameter("csv_logging_enabled").value
        )

        self._csv_path = CSV_PATH
        self._csv_file: Any | None = None
        self._csv_writer: csv.DictWriter | None = None
        self._frame_index = 0
        self._first_timestamp_sec: float | None = None

        self._validate_parameters(model_path=model_path)
        self._config = load_config(config_path)
        self._model = YOLO(str(model_path))
        self._bridge = CvBridge()

        if self._csv_logging_enabled:
            self._initialize_csv()

        self._publisher = self.create_publisher(
            BuoyRelativePosition,
            self._output_topic,
            10,
        )
        self._subscription = self.create_subscription(
            Image,
            self._input_image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Buoy detector started: "
            f"image={self._input_image_topic}, "
            f"output={self._output_topic}, "
            f"frame_id={self._body_frame_id}, "
            f"device={self._device}, "
            f"csv={self._csv_path if self._csv_path else 'disabled'}"
        )

        camera_config = require_mapping(self._config, "camera")
        if bool(camera_config.get("provisional", False)):
            self.get_logger().warning(
                "水中屈折を考慮した暫定カメラ内部パラメータを使用しています。"
                "正規化偏差は利用できますが、ヨー角の絶対値と三次元位置は"
                "実機の水中校正・距離検証後に再確認してください。"
            )

        if not self._position_estimation_enabled:
            self.get_logger().warning(
                "position_estimation_enabled=falseです。"
                "ヨー角は配信しますが、位置座標は"
                "position_valid=false、座標=NaNになります。"
            )

    def _validate_parameters(self, model_path: Path) -> None:
        if not model_path.is_file():
            raise FileNotFoundError(
                f"YOLOモデルがありません: {model_path}"
            )
        if self._imgsz <= 0:
            raise ValueError("imgszは正の整数で指定してください。")
        if not 0.0 <= self._confidence_threshold <= 1.0:
            raise ValueError("confidenceは0～1で指定してください。")
        if self._class_id < 0:
            raise ValueError("class_idは0以上で指定してください。")
        if not self._body_frame_id:
            raise ValueError("body_frame_idを空にはできません。")
    @staticmethod
    def _format_csv_float(value: float | None) -> str:
        """有限な数値だけCSV文字列へ変換し、無効値は空欄にする。"""
        if value is None or not np.isfinite(value):
            return ""
        return f"{float(value):.6f}"

    def _initialize_csv(self) -> None:
        """ノード起動を1試行として固定CSVを新規作成する。"""
        self._csv_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            self._csv_file = self._csv_path.open(
                "w",
                encoding="utf-8-sig",
                newline="",
            )
            self._csv_writer = csv.DictWriter(
                self._csv_file,
                fieldnames=CSV_FIELDS,
            )
            self._csv_writer.writeheader()
            self._csv_file.flush()
        except OSError as error:
            self._csv_file = None
            self._csv_writer = None
            raise RuntimeError(
                f"CSVを初期化できません: {self._csv_path}: {error}"
            ) from error

    def _relative_timestamp_sec(self, image_msg: Image) -> float:
        """最初に受信した画像を0秒とする相対時刻を返す。"""
        stamp = image_msg.header.stamp
        timestamp_sec = (
            float(stamp.sec)
            + float(stamp.nanosec) * 1.0e-9
        )
        if timestamp_sec <= 0.0:
            timestamp_sec = (
                float(self.get_clock().now().nanoseconds) * 1.0e-9
            )

        if self._first_timestamp_sec is None:
            self._first_timestamp_sec = timestamp_sec

        return timestamp_sec - self._first_timestamp_sec

    def _write_csv_row(
        self,
        timestamp_sec: float,
        detected: bool,
        position_valid: bool,
        confidence: float,
        horizontal_offset: float | None,
        vertical_offset: float | None,
        yaw_valid: bool,
        yaw_rad: float | None,
        point_body: np.ndarray | None,
    ) -> None:
        """コード内の推論結果を画像1枚につき1行保存する。"""
        if self._csv_writer is None or self._csv_file is None:
            return

        x_value = ""
        y_value = ""
        z_value = ""
        if (
            position_valid
            and point_body is not None
            and point_body.shape == (3,)
            and np.all(np.isfinite(point_body))
        ):
            x_value = self._format_csv_float(float(point_body[0]))
            y_value = self._format_csv_float(float(point_body[1]))
            z_value = self._format_csv_float(float(point_body[2]))

        yaw_value = ""
        if yaw_valid:
            yaw_value = self._format_csv_float(yaw_rad)

        confidence_value = (
            float(confidence)
            if np.isfinite(confidence)
            else 0.0
        )

        row = {
            "frame_index": self._frame_index,
            "timestamp_sec": f"{timestamp_sec:.6f}",
            "detected": bool(detected),
            "yaw_valid": bool(yaw_valid),
            "position_valid": bool(position_valid),
            "x": x_value,
            "y": y_value,
            "z": z_value,
            "yaw": yaw_value,
            "horizontal_offset": self._format_csv_float(
                horizontal_offset
            ),
            "vertical_offset": self._format_csv_float(
                vertical_offset
            ),
            "confidence": f"{confidence_value:.6f}",
        }

        try:
            self._csv_writer.writerow(row)
            # 各画像更新後にディスクへ反映する。
            self._csv_file.flush()
        except OSError as error:
            self.get_logger().error(
                f"CSVへの書き込みに失敗しました: {error}"
            )
            try:
                self._csv_file.close()
            except OSError:
                pass
            self._csv_file = None
            self._csv_writer = None
        finally:
            self._frame_index += 1

    def _image_callback(self, image_msg: Image) -> None:
        timestamp_sec = self._relative_timestamp_sec(image_msg)

        detected = False
        yaw_valid = False
        position_valid = False
        confidence = 0.0
        normalized_horizontal_offset: float | None = None
        normalized_vertical_offset: float | None = None
        yaw_rad: float | None = None
        point_body: np.ndarray | None = None

        try:
            image = self._bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding="bgr8",
            )
        except CvBridgeError as error:
            self.get_logger().error(
                f"ROS画像をOpenCV画像へ変換できません: {error}"
            )
            if self._csv_logging_enabled:
                self._write_csv_row(
                    timestamp_sec=timestamp_sec,
                    detected=detected,
                    position_valid=position_valid,
                    confidence=confidence,
                    horizontal_offset=normalized_horizontal_offset,
                    vertical_offset=normalized_vertical_offset,
                    yaw_valid=yaw_valid,
                    yaw_rad=yaw_rad,
                    point_body=point_body,
                )
            return

        try:
            results = self._model.predict(
                source=image,
                imgsz=self._imgsz,
                conf=self._confidence_threshold,
                device=self._device,
                classes=[self._class_id],
                max_det=1,
                save=False,
                verbose=False,
            )

            if results:
                (
                    detected,
                    yaw_valid,
                    position_valid,
                    confidence,
                    normalized_horizontal_offset,
                    normalized_vertical_offset,
                    yaw_rad,
                    point_body,
                ) = self._process_result(
                    result=results[0],
                    image_width=int(image.shape[1]),
                    image_height=int(image.shape[0]),
                )
        except Exception as error:
            # 1フレームの失敗でノード全体を停止させない。
            self.get_logger().error(f"画像推論に失敗しました: {error}")

        # publish用メッセージではなく、コード内の計算変数を直接保存する。
        # 検出なし・座標無効の場合も、画像更新1回につき必ず1行追加する。
        if self._csv_logging_enabled:
            self._write_csv_row(
                timestamp_sec=timestamp_sec,
                detected=detected,
                position_valid=position_valid,
                confidence=confidence,
                horizontal_offset=normalized_horizontal_offset,
                vertical_offset=normalized_vertical_offset,
                yaw_valid=yaw_valid,
                yaw_rad=yaw_rad,
                point_body=point_body,
            )

        self._publish_result(
            source_msg=image_msg,
            detected=detected,
            yaw_valid=yaw_valid,
            position_valid=position_valid,
            confidence=confidence,
            normalized_horizontal_offset=normalized_horizontal_offset,
            normalized_vertical_offset=normalized_vertical_offset,
            yaw_rad=yaw_rad,
            point_body=point_body,
        )

    def _process_result(
        self,
        result: Any,
        image_width: int,
        image_height: int,
    ) -> tuple[
        bool,
        bool,
        bool,
        float,
        float | None,
        float | None,
        float | None,
        np.ndarray | None,
    ]:
        if result.boxes is None or len(result.boxes) == 0:
            return False, False, False, 0.0, None, None, None, None

        confidences = result.boxes.conf.detach().cpu().numpy()
        best_index = int(np.argmax(confidences))
        confidence = float(confidences[best_index])
        box_values = (
            result.boxes.xyxy[best_index]
            .detach()
            .cpu()
            .numpy()
            .astype(np.float64)
        )

        bbox = BoundingBox(
            x1=float(box_values[0]),
            y1=float(box_values[1]),
            x2=float(box_values[2]),
            y2=float(box_values[3]),
        )

        try:
            bbox.validate()
        except ValueError as error:
            self.get_logger().debug(f"BBoxを無効化しました: {error}")
            return True, False, False, confidence, None, None, None, None

        normalized_horizontal_offset: float | None = None
        try:
            normalized_horizontal_offset = (
                calculate_normalized_horizontal_offset(
                    bbox=bbox,
                    image_width=image_width,
                )
            )
        except ValueError as error:
            self.get_logger().debug(
                f"正規化水平偏差を無効化しました: {error}"
            )

        normalized_vertical_offset: float | None = None
        try:
            normalized_vertical_offset = (
                calculate_normalized_vertical_offset(
                    bbox=bbox,
                    image_height=image_height,
                )
            )
        except ValueError as error:
            self.get_logger().debug(
                f"正規化垂直偏差を無効化しました: {error}"
            )

        yaw_valid = False
        yaw_rad: float | None = None
        try:
            yaw_rad = calculate_body_yaw(
                bbox=bbox,
                image_width=image_width,
                image_height=image_height,
                config=self._config,
            )
            yaw_valid = bool(np.isfinite(yaw_rad))
        except ValueError as error:
            self.get_logger().debug(f"ヨー角を無効化しました: {error}")

        if not self._position_estimation_enabled:
            return (
                True,
                yaw_valid,
                False,
                confidence,
                normalized_horizontal_offset,
                normalized_vertical_offset,
                yaw_rad,
                None,
            )

        border_margin_px = int(
            require_mapping(self._config, "validation")[
                "border_margin_px"
            ]
        )
        if bbox.touches_image_border(
            image_width=image_width,
            image_height=image_height,
            margin_px=border_margin_px,
        ):
            self.get_logger().debug(
                "BBoxが画像端に接触したため位置座標を無効化しました。"
            )
            return (
                True,
                yaw_valid,
                False,
                confidence,
                normalized_horizontal_offset,
                normalized_vertical_offset,
                yaw_rad,
                None,
            )

        try:
            point_body = calculate_body_position(
                bbox=bbox,
                image_width=image_width,
                image_height=image_height,
                config=self._config,
            )
        except ValueError as error:
            self.get_logger().debug(f"座標推定を無効化しました: {error}")
            return (
                True,
                yaw_valid,
                False,
                confidence,
                normalized_horizontal_offset,
                normalized_vertical_offset,
                yaw_rad,
                None,
            )

        return (
            True,
            yaw_valid,
            True,
            confidence,
            normalized_horizontal_offset,
            normalized_vertical_offset,
            yaw_rad,
            point_body,
        )

    def _publish_result(
        self,
        source_msg: Image,
        detected: bool,
        yaw_valid: bool,
        position_valid: bool,
        confidence: float,
        normalized_horizontal_offset: float | None,
        normalized_vertical_offset: float | None,
        yaw_rad: float | None,
        point_body: np.ndarray | None,
    ) -> None:
        output = BuoyRelativePosition()
        output.header.stamp = source_msg.header.stamp
        output.header.frame_id = self._body_frame_id
        output.detected = detected
        output.yaw_valid = yaw_valid
        output.position_valid = position_valid
        output.confidence = float(confidence)

        invalid_value = float("nan")
        output.normalized_horizontal_offset = invalid_value
        output.normalized_vertical_offset = invalid_value
        output.relative_buoy_yaw_rad = invalid_value
        output.relative_buoy_x_m = invalid_value
        output.relative_buoy_y_m = invalid_value
        output.relative_buoy_z_m = invalid_value

        if (
            detected
            and normalized_horizontal_offset is not None
            and np.isfinite(normalized_horizontal_offset)
        ):
            output.normalized_horizontal_offset = float(
                np.clip(normalized_horizontal_offset, -1.0, 1.0)
            )

        if (
            detected
            and normalized_vertical_offset is not None
            and np.isfinite(normalized_vertical_offset)
        ):
            output.normalized_vertical_offset = float(
                np.clip(normalized_vertical_offset, -1.0, 1.0)
            )

        if yaw_valid and yaw_rad is not None and np.isfinite(yaw_rad):
            output.relative_buoy_yaw_rad = float(yaw_rad)
        else:
            output.yaw_valid = False

        if position_valid and point_body is not None:
            if point_body.shape != (3,) or not np.all(
                np.isfinite(point_body)
            ):
                output.position_valid = False
                self.get_logger().error(
                    "算出した機体座標が不正なためNaNを配信します。"
                )
            else:
                output.relative_buoy_x_m = float(point_body[0])
                output.relative_buoy_y_m = float(point_body[1])
                output.relative_buoy_z_m = float(point_body[2])

        self._publisher.publish(output)

    def destroy_node(self) -> bool:
        """終了時にCSVを確実に保存して閉じる。"""
        if self._csv_file is not None:
            try:
                self._csv_file.flush()
                self._csv_file.close()
            except OSError as error:
                self.get_logger().error(
                    f"CSVを閉じる際にエラーが発生しました: {error}"
                )
            finally:
                self._csv_file = None
                self._csv_writer = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: BuoyDetectorNode | None = None

    try:
        node = BuoyDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(f"ノードを終了します: {error}")
        else:
            print(f"ERROR: {error}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
