"""Unit tests for pure MAVLink command conversion helpers."""

import math
import unittest

from mavlink_driver.conversions import (
    angle_to_pwm,
    axis_to_pwm,
    euler_to_quaternion,
    radians_to_degrees,
)


class TestConversions(unittest.TestCase):
    """Verify deterministic conversions independent of ROS and MAVLink I/O."""

    def test_axis_to_pwm_clamps_and_inverts(self):
        """Convert signed commands with saturation and an inverted axis."""
        self.assertEqual(axis_to_pwm(80.0, 40.0, False, 1300, 1500, 1700), 1700)
        self.assertEqual(axis_to_pwm(-80.0, 40.0, False, 1300, 1500, 1700), 1300)
        self.assertEqual(axis_to_pwm(20.0, 40.0, True, 1300, 1500, 1700), 1400)

    def test_axis_to_pwm_rejects_invalid_limit(self):
        """Reject an axis configuration that cannot be normalized."""
        with self.assertRaises(ValueError):
            axis_to_pwm(0.0, 0.0, False, 1300, 1500, 1700)

    def test_angle_to_pwm_clamps(self):
        """Clamp camera angles to the configured PWM range."""
        self.assertEqual(angle_to_pwm(-1.0, 0.0, 180.0, 1100, 1900), 1100)
        self.assertEqual(angle_to_pwm(90.0, 0.0, 180.0, 1100, 1900), 1500)
        self.assertEqual(angle_to_pwm(360.0, 0.0, 180.0, 1100, 1900), 1900)

    def test_euler_to_quaternion_identity(self):
        """Produce the identity quaternion for zero Euler angles."""
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, 0.0)
        self.assertEqual((qx, qy, qz), (0.0, 0.0, 0.0))
        self.assertTrue(math.isclose(qw, 1.0))

    def test_radians_to_degrees(self):
        """Convert the radians unit used by MAVLink attitude messages."""
        self.assertTrue(math.isclose(radians_to_degrees(math.pi), 180.0))
