/**
 * @file tilt_servo.hpp
 * @brief servo driver for camera tilt
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details カメラのチルト用サーボモータの信号をSensor基板に送信
 **************************************************************/

#ifndef _TILT_SERVO_HPP
#define _TILT_SERVO_HPP

#include <driver_msgs/msg/int32_stamped.hpp>
#include <proto_files/conversion_driver_msgs__Int32Stamped.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @namespace sensor_board_driver
 * @brief driver for sensor_board
 */
namespace sensor_board_driver
{

/**
 * @brief TiltServo class
 */
class TiltServo : public rclcpp::Node
{
public:
  /**
   * @brief Protolink & ROS-topic settng
   * @details Protolink setting and Define a subscriber
   */
  explicit TiltServo();

private:
  boost::asio::io_context io_context_;
  std::string mcu_ip_addr;  // of microcontroller, etc.
  uint16_t mcu_port;        // same as above
  uint16_t this_port;       // of the computer executing this code

  using protoInt32Stamped = protolink__driver_msgs__Int32Stamped::driver_msgs__Int32Stamped;
  std::shared_ptr<protolink::udp_protocol::Publisher<protoInt32Stamped>> protolink_publisher_;

  rclcpp::Subscription<driver_msgs::msg::Int32Stamped>::SharedPtr sub_;
};

}  // namespace sensor_board_driver

#endif  // !_TILT_SERVO_HPP
