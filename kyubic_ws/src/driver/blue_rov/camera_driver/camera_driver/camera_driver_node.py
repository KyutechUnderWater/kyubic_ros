"""ROS 2 driver that receives the BlueROV video stream over UDP and republishes it."""

from __future__ import annotations

import threading
import time
from typing import Optional

import gi
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402 (must follow gi.require_version)


class CameraDriver(Node):
    """Receive the BlueROV RTP/H264 video stream and publish it as sensor_msgs/Image.

    The node exclusively owns the GStreamer pipeline for one BlueROV camera.
    It decodes the incoming UDP stream and republishes frames on a ROS topic,
    dropping frames as needed to cap the publish rate below the source rate.
    """

    def __init__(self) -> None:
        """Initialize parameters, the ROS publisher, and the GStreamer pipeline."""
        super().__init__("camera_driver")

        self._udp_port = self.declare_parameter("udp_port", 5600).value
        self._max_fps = self.declare_parameter("max_fps", 15.0).value
        self._frame_id = self.declare_parameter("frame_id", "bluerov_camera").value

        self._bridge = CvBridge()
        self._publisher = self.create_publisher(Image, "image_raw", qos_profile_sensor_data)

        self._min_frame_interval_s = 1.0 / self._max_fps if self._max_fps > 0.0 else 0.0
        self._last_publish_time = 0.0

        Gst.init(None)
        self._pipeline = self._build_pipeline(self._udp_port)
        appsink = self._pipeline.get_by_name("sink")
        appsink.connect("new-sample", self._on_new_sample)

        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        self._loop = GLib.MainLoop()
        self._loop_thread = threading.Thread(target=self._loop.run, daemon=True)
        self._loop_thread.start()

        self._pipeline.set_state(Gst.State.PLAYING)

    def _build_pipeline(self, udp_port: int) -> Gst.Pipeline:
        """Construct the receive pipeline for one BlueROV camera.

        Args:
            udp_port: UDP port carrying the RTP/H264 stream from the vehicle.

        Returns:
            Gst.Pipeline: The constructed pipeline, not yet playing.
        """
        pipeline_str = (
            f"udpsrc port={udp_port} ! "
            "application/x-rtp, payload=96 ! "
            "rtph264depay ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
        )
        return Gst.parse_launch(pipeline_str)

    def _on_new_sample(self, sink: Gst.Element) -> Gst.FlowReturn:
        """Pull a decoded frame from the sink and publish it, honoring the FPS cap."""
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        now = time.monotonic()
        if now - self._last_publish_time < self._min_frame_interval_s:
            return Gst.FlowReturn.OK

        buffer = sample.get_buffer()
        structure = sample.get_caps().get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")

        ok, mapinfo = buffer.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR
        try:
            frame = np.frombuffer(mapinfo.data, dtype=np.uint8).reshape((height, width, 3))
            msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        finally:
            buffer.unmap(mapinfo)

        self._last_publish_time = now
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._publisher.publish(msg)
        return Gst.FlowReturn.OK

    def _on_bus_message(self, bus: Gst.Bus, message: Gst.Message) -> None:
        """Log pipeline errors and end-of-stream events from the GStreamer bus."""
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            self.get_logger().error(f"GStreamerパイプラインエラー: {error} ({debug})")
        elif message.type == Gst.MessageType.EOS:
            self.get_logger().warning("GStreamerパイプラインがEOSを受信しました。")

    def destroy_node(self) -> bool:
        """Stop the GStreamer pipeline and GLib loop before tearing down the node."""
        self._pipeline.set_state(Gst.State.NULL)
        self._loop.quit()
        self._loop_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    """Run the camera driver node until shutdown."""
    rclpy.init(args=args)
    node = CameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
