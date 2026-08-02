"""Unit tests for RC override channel assembly."""

import unittest

from mavlink_driver.mavlink_driver import RC_RELEASE, MavlinkDriver


class TestRcChannels(unittest.TestCase):
    """Verify inactive control releases thruster channels instead of forcing PWM."""

    @classmethod
    def setUpClass(cls) -> None:
        import rclpy

        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        import rclpy

        rclpy.shutdown()

    def test_inactive_control_releases_thruster_axes(self) -> None:
        node = MavlinkDriver()
        axis_values = {name: 0.0 for name in node._axes}
        channels = node._build_rc_channels(False, axis_values)
        for axis in node._axes.values():
            self.assertEqual(channels[axis.channel - 1], RC_RELEASE)
        node.destroy_node()

    def test_active_zero_command_uses_neutral_pwm(self) -> None:
        node = MavlinkDriver()
        axis_values = {name: 0.0 for name in node._axes}
        channels = node._build_rc_channels(True, axis_values)
        for axis in node._axes.values():
            self.assertEqual(channels[axis.channel - 1], node._pwm_neutral)
        node.destroy_node()
