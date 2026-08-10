"""One-shot body-frame step/velocity target commander for p_pid_controller gain tuning."""
from __future__ import annotations

import argparse
import math

import rclpy
from localization_msgs.msg import Odometry
from p_pid_controller_msgs.msg import Targets
from rclpy.node import Node

AXES = ("x", "y", "z", "yaw")


class StepCommander(Node):
    """Publish a single Targets message that steps or drives one axis.

    Reads the current odometry once and holds every other axis at its
    current value.

    Position mode (default): offsets the requested axis by ``step`` (meters
    for x/y/z, degrees for yaw). x/y are rotated by the current yaw so the
    step lands along the vehicle's body-frame surge/lateral direction
    regardless of heading, mirroring the world-to-body rotation applied in
    test_p_pid.cpp.

    Velocity mode (--velocity): bypasses the outer position P loop for the
    requested axis and drives its inner velocity loop directly with a
    constant target velocity of ``step`` (m/s for x/y/z, deg/s for yaw), via
    Targets.velocity_axis_mask / Targets.twist. Unlike position mode, x/y are
    NOT rotated here: test_p_pid.cpp now rotates the world-frame velocity
    measurement into body frame internally and runs the x/y loops entirely in
    body frame, so twist.x/twist.y are body-frame surge/sway targets as-is,
    and each axis can be driven independently without affecting the other.
    """

    def __init__(
        self, axis: str, step: float, velocity: bool, targets_topic: str, odom_topic: str
    ) -> None:
        super().__init__("step_commander")
        self._axis = axis
        self._step = step
        self._velocity = velocity
        self._pub = self.create_publisher(Targets, targets_topic, 1)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 1)

    def _on_odom(self, msg: Odometry) -> None:
        x0 = msg.pose.position.x
        y0 = msg.pose.position.y
        z0 = msg.pose.position.z_depth
        roll0 = msg.pose.orientation.x
        yaw0 = msg.pose.orientation.z
        yaw_rad = math.radians(yaw0)

        # Body-frame (bx, by) quantity (position step or velocity), rotated
        # into world frame by current yaw. Same rotation for both modes: only
        # what it represents (a step vs. a velocity) differs.
        bx, by = 0.0, 0.0
        if self._axis == "x":
            bx = self._step
        elif self._axis == "y":
            by = self._step

        dx = bx * math.cos(yaw_rad) - by * math.sin(yaw_rad)
        dy = bx * math.sin(yaw_rad) + by * math.cos(yaw_rad)

        target = Targets()
        target.z_mode = Targets.Z_MODE_DEPTH
        # Hold every axis at its current value by default; velocity mode
        # overrides the selected axis below via velocity_axis_mask/twist
        # instead, and non-selected axes stay in position-hold mode.
        target.pose.x = x0
        target.pose.y = y0
        target.pose.z = z0
        target.pose.roll = roll0
        target.pose.yaw = yaw0

        if self._velocity:
            # Body-frame target as-is; test_p_pid.cpp does the world<->body rotation
            # internally now, so each axis is independent (no need to also drive the
            # other of x/y).
            if self._axis == "x":
                target.velocity_axis_mask = Targets.VELOCITY_AXIS_X
                target.twist.x = self._step
            elif self._axis == "y":
                target.velocity_axis_mask = Targets.VELOCITY_AXIS_Y
                target.twist.y = self._step
            elif self._axis == "z":
                target.velocity_axis_mask = Targets.VELOCITY_AXIS_Z
                target.twist.z = self._step
            elif self._axis == "yaw":
                target.velocity_axis_mask = Targets.VELOCITY_AXIS_YAW
                target.twist.yaw = self._step
            self._pub.publish(target)
            self.get_logger().info(
                f"[velocity mode] axis={self._axis} target_velocity={self._step} "
                f"(twist.x={target.twist.x:.3f} twist.y={target.twist.y:.3f} "
                f"twist.z={target.twist.z:.3f} twist.yaw={target.twist.yaw:.3f})"
            )
        else:
            target.pose.x = x0 + dx
            target.pose.y = y0 + dy
            if self._axis == "z":
                target.pose.z = z0 + self._step
            elif self._axis == "yaw":
                target.pose.yaw = yaw0 + self._step
            self._pub.publish(target)
            self.get_logger().info(
                f"axis={self._axis} step={self._step} "
                f"x:{x0:.2f}->{target.pose.x:.2f} y:{y0:.2f}->{target.pose.y:.2f} "
                f"z:{z0:.2f}->{target.pose.z:.2f} yaw:{yaw0:.1f}->{target.pose.yaw:.1f}"
            )
        rclpy.shutdown()


def _parse_args(args: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("axis", choices=AXES, help="Axis to command (x/y/z in meters, yaw in degrees)")
    parser.add_argument(
        "step", type=float, help="Position step (default mode) or target velocity (--velocity)"
    )
    parser.add_argument(
        "--velocity",
        action="store_true",
        help="Bypass the outer position P loop and drive the inner velocity loop directly "
        "with a constant target velocity (m/s, or deg/s for yaw), instead of a position step.",
    )
    parser.add_argument(
        "--targets-topic",
        default="/rt_pose_plotter/targets",
        help="Targets topic to publish to (default: %(default)s)",
    )
    parser.add_argument(
        "--odom-topic",
        default="/localization/odom",
        help="Odometry topic to read the current pose from (default: %(default)s)",
    )
    return parser.parse_args(args=args)


def main(args: list[str] | None = None) -> None:
    """Run the step commander node until it publishes one target, then exit."""
    parsed = _parse_args(args)
    rclpy.init()
    node = StepCommander(
        parsed.axis, parsed.step, parsed.velocity, parsed.targets_topic, parsed.odom_topic
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
