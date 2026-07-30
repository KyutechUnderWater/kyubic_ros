from __future__ import annotations

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
    estimate_range_inverse_scale,
    estimate_range_pinhole_area,
    project_bbox_center_to_camera,
    transform_camera_to_body,
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
    """ブイ位置推定用YAMLを読み込む。"""
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

    if range_model == "pinhole_area":
        range_z_m = estimate_range_pinhole_area(
            bbox=bbox,
            intrinsics=intrinsics,
            buoy_width_m=float(range_config["buoy_width_m"]),
            buoy_height_m=float(range_config["buoy_height_m"]),
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

    min_range_m = float(range_config["min_range_m"])
    max_range_m = float(range_config["max_range_m"])
    if not min_range_m <= range_z_m <= max_range_m:
        raise ValueError(
            f"推定距離{range_z_m:.3f} mが有効範囲外です。"
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
    """ROS画像からブイを検出し、base_link座標を配信する。"""

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
        self.declare_parameter("input_image_topic", "/camera/image_raw")
        self.declare_parameter(
            "output_topic",
            "/buoy/relative_position",
        )
        self.declare_parameter(
            "body_frame_id",
            "base_link",
        )
        self.declare_parameter("imgsz", 320)
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("class_id", 0)
        self.declare_parameter("position_estimation_enabled", True)

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

        self._validate_parameters(model_path=model_path)
        self._config = load_config(config_path)
        self._model = YOLO(str(model_path))
        self._bridge = CvBridge()

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
            f"device={self._device}"
        )

        if not self._position_estimation_enabled:
            self.get_logger().warning(
                "position_estimation_enabled=falseです。"
                "検出時もposition_valid=false、座標=NaNで配信します。"
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

    def _image_callback(self, image_msg: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding="bgr8",
            )
        except CvBridgeError as error:
            self.get_logger().error(
                f"ROS画像をOpenCV画像へ変換できません: {error}"
            )
            return

        detected = False
        position_valid = False
        confidence = 0.0
        point_body: np.ndarray | None = None

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
                    position_valid,
                    confidence,
                    point_body,
                ) = self._process_result(
                    result=results[0],
                    image_width=int(image.shape[1]),
                    image_height=int(image.shape[0]),
                )
        except Exception as error:
            # 1フレームの失敗でノード全体を停止させない。
            self.get_logger().error(f"画像推論に失敗しました: {error}")

        self._publish_result(
            source_msg=image_msg,
            detected=detected,
            position_valid=position_valid,
            confidence=confidence,
            point_body=point_body,
        )

    def _process_result(
        self,
        result: Any,
        image_width: int,
        image_height: int,
    ) -> tuple[bool, bool, float, np.ndarray | None]:
        if result.boxes is None or len(result.boxes) == 0:
            return False, False, 0.0, None

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
            return True, False, confidence, None

        if not self._position_estimation_enabled:
            return True, False, confidence, None

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
                "BBoxが画像端に接触したため座標を無効化しました。"
            )
            return True, False, confidence, None

        try:
            point_body = calculate_body_position(
                bbox=bbox,
                image_width=image_width,
                image_height=image_height,
                config=self._config,
            )
        except ValueError as error:
            self.get_logger().debug(f"座標推定を無効化しました: {error}")
            return True, False, confidence, None

        return True, True, confidence, point_body

    def _publish_result(
        self,
        source_msg: Image,
        detected: bool,
        position_valid: bool,
        confidence: float,
        point_body: np.ndarray | None,
    ) -> None:
        output = BuoyRelativePosition()
        output.header.stamp = source_msg.header.stamp
        output.header.frame_id = self._body_frame_id
        output.detected = detected
        output.position_valid = position_valid
        output.confidence = float(confidence)

        invalid_value = float("nan")
        output.relative_buoy_x_m = invalid_value
        output.relative_buoy_y_m = invalid_value
        output.relative_buoy_z_m = invalid_value

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
