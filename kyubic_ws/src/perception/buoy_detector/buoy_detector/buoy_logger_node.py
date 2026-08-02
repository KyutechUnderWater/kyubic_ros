"""ROS 2 node that logs camera frames and YOLO buoy detections at a low rate."""

from __future__ import annotations

import csv
from pathlib import Path

from buoy_interfaces.msg import BuoyRelativePosition
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

CSV_HEADER = [
    "stamp_sec",
    "stamp_nanosec",
    "image_filename",
    "detected",
    "position_valid",
    "confidence",
    "relative_buoy_x_m",
    "relative_buoy_y_m",
    "relative_buoy_z_m",
]


class BuoyLoggerNode(Node):
    """Periodically save the latest camera frame and buoy detection result.

    The node subscribes to the camera image and the buoy detector output
    independently and keeps only the most recent message of each. A timer
    running at ``save_rate_hz`` writes the latest image to disk and appends
    one row to a CSV file, decoupling the saved rate from the (much higher)
    publish rate of the camera and detector.
    """

    def __init__(self) -> None:
        """Initialize parameters, I/O paths, subscriptions, and the save timer."""
        super().__init__("buoy_logger")

        self.declare_parameter("input_image_topic", "/camera/image_raw")
        self.declare_parameter(
            "input_detection_topic", "/buoy/relative_position"
        )
        self.declare_parameter("output_dir", "~/buoy_logs")
        self.declare_parameter("save_rate_hz", 1.5)
        self.declare_parameter("image_format", "jpg")

        self._input_image_topic = str(
            self.get_parameter("input_image_topic").value
        )
        self._input_detection_topic = str(
            self.get_parameter("input_detection_topic").value
        )
        self._output_dir = Path(
            str(self.get_parameter("output_dir").value)
        ).expanduser()
        self._save_rate_hz = float(self.get_parameter("save_rate_hz").value)
        self._image_format = str(self.get_parameter("image_format").value)

        self._validate_parameters()

        self._images_dir = self._output_dir / "images"
        self._images_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = self._output_dir / "detections.csv"
        self._init_csv()

        self._bridge = CvBridge()
        self._latest_image: Image | None = None
        self._latest_detection: BuoyRelativePosition | None = None
        self._pending_save = False

        self._image_subscription = self.create_subscription(
            Image,
            self._input_image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._detection_subscription = self.create_subscription(
            BuoyRelativePosition,
            self._input_detection_topic,
            self._detection_callback,
            10,
        )
        self._timer = self.create_timer(
            1.0 / self._save_rate_hz, self._save_callback
        )

        self.get_logger().info(
            "Buoy logger started: "
            f"image={self._input_image_topic}, "
            f"detection={self._input_detection_topic}, "
            f"output_dir={self._output_dir}, "
            f"save_rate_hz={self._save_rate_hz}"
        )

    def _validate_parameters(self) -> None:
        if self._save_rate_hz <= 0.0:
            raise ValueError("save_rate_hzは正の数で指定してください。")
        if self._image_format not in ("jpg", "png"):
            raise ValueError("image_formatは'jpg'または'png'です。")

    def _init_csv(self) -> None:
        if self._csv_path.is_file():
            return
        with self._csv_path.open("w", newline="", encoding="utf-8") as file:
            csv.writer(file).writerow(CSV_HEADER)

    def _image_callback(self, image_msg: Image) -> None:
        self._latest_image = image_msg
        self._pending_save = True

    def _detection_callback(self, detection_msg: BuoyRelativePosition) -> None:
        self._latest_detection = detection_msg

    def _save_callback(self) -> None:
        if not self._pending_save or self._latest_image is None:
            return
        self._pending_save = False

        image_msg = self._latest_image
        detection_msg = self._latest_detection

        try:
            image = self._bridge.imgmsg_to_cv2(
                image_msg, desired_encoding="bgr8"
            )
        except CvBridgeError as error:
            self.get_logger().error(
                f"ROS画像をOpenCV画像へ変換できません: {error}"
            )
            return

        stamp_sec = int(image_msg.header.stamp.sec)
        stamp_nanosec = int(image_msg.header.stamp.nanosec)
        image_filename = f"{stamp_sec}_{stamp_nanosec:09d}.{self._image_format}"

        image_path = self._images_dir / image_filename
        if not cv2.imwrite(str(image_path), image):
            self.get_logger().error(f"画像の保存に失敗しました: {image_path}")
            return

        self._append_csv_row(
            stamp_sec=stamp_sec,
            stamp_nanosec=stamp_nanosec,
            image_filename=image_filename,
            detection_msg=detection_msg,
        )

    def _append_csv_row(
        self,
        stamp_sec: int,
        stamp_nanosec: int,
        image_filename: str,
        detection_msg: BuoyRelativePosition | None,
    ) -> None:
        if detection_msg is None:
            row = [stamp_sec, stamp_nanosec, image_filename, "", "", "", "", "", ""]
        else:
            row = [
                stamp_sec,
                stamp_nanosec,
                image_filename,
                detection_msg.detected,
                detection_msg.position_valid,
                detection_msg.confidence,
                detection_msg.relative_buoy_x_m,
                detection_msg.relative_buoy_y_m,
                detection_msg.relative_buoy_z_m,
            ]

        with self._csv_path.open("a", newline="", encoding="utf-8") as file:
            csv.writer(file).writerow(row)


def main(args: list[str] | None = None) -> None:
    """Run the buoy logger node until shutdown."""
    rclpy.init(args=args)
    node: BuoyLoggerNode | None = None

    try:
        node = BuoyLoggerNode()
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
