/**
 * @file imu_reset.hpp
 * @brief imu hardware reset driver
 * @author R.Ohnishi
 * @date 2025/11/21
 *
 * @details IMUのハードウェアリセット信号をSensor基板に送信
 *********************************************************/

#ifndef _IMU_RESET_HPP
#define _IMU_RESET_HPP

#include <driver_msgs/msg/bool_stamped.hpp>
#include <memory>
#include <proto_files/conversion_driver_msgs__BoolStamped.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @namespace driver::sensors_esp32_driver
 * @brief driver for sensors_esp32
 */
namespace driver::sensors_esp32_driver
{

/**
 * @brief ImuReset class
 */
class ImuReset : public rclcpp::Node
{
public:
  /**
   * @brief protolink & ROS-topic settng
   * @details protolink setting and Define a subscriber
   */
  explicit ImuReset(const rclcpp::NodeOptions & options);

private:
  std::string mcu_ip_addr;  // of microcontroller, etc.
  uint16_t mcu_port;        // same as above
  uint16_t this_port;       // of the computer executing this code

  protolink::IoContext io_context_;
  std::shared_ptr<protolink::udp_protocol::soket> sock_;

  using protoBoolStamped = protolink__driver_msgs__BoolStamped::driver_msgs__BoolStamped;
  std::shared_ptr<protolink::udp_protocol::Publisher<protoBoolStamped>> protolink_publisher_;

  rclcpp::Subscription<driver_msgs::msg::BoolStamped>::SharedPtr sub_;
};

}  // namespace driver::sensors_esp32_driver

#endif  // !_IMU_RESET_HPP
