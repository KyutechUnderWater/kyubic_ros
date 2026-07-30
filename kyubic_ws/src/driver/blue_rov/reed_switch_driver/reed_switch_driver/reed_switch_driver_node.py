"""ROS 2 driver that publishes a one-shot mission-start trigger from a reed switch."""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from driver_msgs.msg import LED, BoolStamped
from rclpy.node import Node

try:
    from gpiozero import LED as GpioLED
    from gpiozero import Button

    GPIOZERO_AVAILABLE = True
except ImportError:
    GPIOZERO_AVAILABLE = False


class ReedSwitchDriver(Node):
    """Publish a one-shot trigger and flash an LED when the reed switch fires.

    This node only reports the physical event (competition "mission start"
    magnet switch). Any start-delay logic (e.g. waiting one minute before the
    behavior tree actually engages the thrusters) belongs downstream, in the
    node that subscribes to this trigger.
    """

    def __init__(self) -> None:
        """Configure GPIO pins and the trigger publisher."""
        super().__init__("reed_switch_driver")

        self._reed_switch_pin = self.declare_parameter("reed_switch_pin", 17).value
        self._led_pin = self.declare_parameter("led_pin", 27).value
        self._debounce_s = self.declare_parameter("debounce_s", 0.05).value
        self._led_flash_duration_s = self.declare_parameter("led_flash_duration_s", 0.2).value
        # 磁石を近づけると回路が閉じる(pressed)前提。
        # trigger_on_press=false(既定): 磁石を抜いて回路が開いた瞬間にトリガー。
        # trigger_on_press=true: 磁石を近づけて回路が閉じた瞬間にトリガー。
        self._trigger_on_press = bool(self.declare_parameter("trigger_on_press", False).value)
        # BlueROV本体のライト(mavlink_driver経由)を一瞬光らせるためのPWM値。
        self._blue_led_on_pwm = self.declare_parameter("blue_led_on_pwm", 1900).value
        self._blue_led_off_pwm = self.declare_parameter("blue_led_off_pwm", 1100).value

        self._validate_parameters()

        self._publisher = self.create_publisher(BoolStamped, "mission_start_trigger", 10)
        self._blue_led_publisher = self.create_publisher(LED, "led", 10)
        self._blue_led_off_timer: Optional[threading.Timer] = None

        if not GPIOZERO_AVAILABLE:
            raise RuntimeError(
                "gpiozero is required to run reed_switch_driver on real hardware. "
                "Install python3-gpiozero (and a pin factory such as python3-lgpio)."
            )

        self._reed_switch: Optional[Button] = Button(
            self._reed_switch_pin,
            pull_up=True,
            bounce_time=self._debounce_s,
        )
        self._led: Optional[GpioLED] = GpioLED(self._led_pin)

        if self._trigger_on_press:
            self._reed_switch.when_pressed = self._on_trigger
        else:
            self._reed_switch.when_released = self._on_trigger

        self.get_logger().info(
            "Reed switch driver ready: "
            f"reed_switch_pin={self._reed_switch_pin}, led_pin={self._led_pin}, "
            f"trigger_on_press={self._trigger_on_press}"
        )

    def _validate_parameters(self) -> None:
        if self._reed_switch_pin == self._led_pin:
            raise ValueError("reed_switch_pin and led_pin must be different GPIO pins.")
        if self._reed_switch_pin < 0 or self._led_pin < 0:
            raise ValueError("GPIO pin numbers must be non-negative.")
        if self._debounce_s < 0.0:
            raise ValueError("debounce_s must be non-negative.")
        if self._led_flash_duration_s <= 0.0:
            raise ValueError("led_flash_duration_s must be positive.")
        if not 1100 <= self._blue_led_on_pwm <= 1900:
            raise ValueError("blue_led_on_pwm must be in the range 1100..1900.")
        if not 1100 <= self._blue_led_off_pwm <= 1900:
            raise ValueError("blue_led_off_pwm must be in the range 1100..1900.")

    def _publish_blue_led(self, pwm: int) -> None:
        message = LED()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status.id = 0
        message.left = pwm
        message.right = pwm
        self._blue_led_publisher.publish(message)

    def _on_trigger(self) -> None:
        """Flash the Pi indicator LED and BlueROV's own lights, and publish the trigger."""
        self._led.blink(on_time=self._led_flash_duration_s, off_time=0.01, n=1, background=True)

        if self._blue_led_off_timer is not None:
            self._blue_led_off_timer.cancel()
        self._publish_blue_led(self._blue_led_on_pwm)
        self._blue_led_off_timer = threading.Timer(
            self._led_flash_duration_s, self._publish_blue_led, args=(self._blue_led_off_pwm,)
        )
        self._blue_led_off_timer.start()

        message = BoolStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "reed_switch"
        message.status.id = 0
        message.data = True
        self._publisher.publish(message)
        self.get_logger().info("Reed switch triggered: mission_start_trigger published.")

    def close_gpio(self) -> None:
        """Release GPIO resources on shutdown."""
        if self._blue_led_off_timer is not None:
            self._blue_led_off_timer.cancel()
        if self._reed_switch is not None:
            self._reed_switch.close()
        if self._led is not None:
            self._led.close()


def main(args: Optional[list] = None) -> None:
    """Run the reed switch driver node until shutdown."""
    rclpy.init(args=args)
    node: Optional[ReedSwitchDriver] = None

    try:
        node = ReedSwitchDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_gpio()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
