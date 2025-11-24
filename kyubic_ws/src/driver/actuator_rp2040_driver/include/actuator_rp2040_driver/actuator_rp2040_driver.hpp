/**
 * @file actuator_rp2040_driver.hpp
 * @brief Driver for rp2040 on Actuator board
 * @author K.Fujimoto
 * @date 2025/11/21
 *
 * @details スラスターの出力値を受け取り，制御マイコンに命令する
 */

#ifndef _ACTUATOR_RP2040_DRIVER_HPP
#define _ACTUATOR_RP2040_DRIVER_HPP

#include <driver_msgs/msg/actuator.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <proto_files/conversion_driver_msgs__Actuator.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <timer/timeout.hpp>

namespace actuator_rp2040_driver
{

/**
 * @brief actuator class with thruster mixing logic
 */
class ActuatorRP2040 : public rclcpp::Node
{
public:
  /**
   * @brief Protolink & ROS-topic setting
   */
  explicit ActuatorRP2040(const rclcpp::NodeOptions & options);

private:
  // Serial settings
  std::string portname;
  uint32_t baudrate;

  // Thruster mixing parameters
  static constexpr size_t NUM_THRUSTERS = 6;
  float max_thrust;
  float max_thrust_per;

  // Robot physical parameters (Defined in header as requested)
  static constexpr float theta_h = 30.0f * M_PI / 180.0f;  //[rad]
  static constexpr float theta_v = 30.0f * M_PI / 180.0f;  //[rad]
  static constexpr float dist_hx = 0.19f;                  // x-axis (horizontal)
  static constexpr float dist_hy = 0.14f;                  // y-axis (horizontal)
  static constexpr float dist_vy = 0.19f;                  // x-axis (vertical)
  static constexpr float dist_vz = 0.14f;                  // y-axis (vertical)

  // Scaling factors for mixing (const members, initialized in constructor)
  const float f_x_scale;
  const float f_y_scale;
  const float f_z_scale;
  const float t_x_scale;
  const float t_z_scale;

  // Heartbeat settings
  bool heartbeat{false};
  double timeout_sec;
  std::shared_ptr<timer::Timeout> timeout_;

  // Protolink members
  protolink::IoContext io_context_;
  std::shared_ptr<protolink::serial_protocol::port> port_;
  using ProtoActuator = protolink__driver_msgs__Actuator::driver_msgs__Actuator;
  std::shared_ptr<protolink::serial_protocol::Publisher<ProtoActuator>> protolink_publisher_;

  driver_msgs::msg::Actuator msg_buffer_;

  // ROS Publishers & Subscribers
  rclcpp::Publisher<driver_msgs::msg::Thruster>::SharedPtr thruster_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<driver_msgs::msg::LED>::SharedPtr led_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_sub_;

  // Callbacks
  void wrench_callback(geometry_msgs::msg::WrenchStamped::SharedPtr _msg);
  void led_callback(driver_msgs::msg::LED::SharedPtr _msg);
  void heartbeat_callback(std_msgs::msg::Bool::SharedPtr _msg);

  // Helper functions
  std::array<float, NUM_THRUSTERS> _wrench2thrusts(
    float f_x, float f_y, float f_z, float t_x, float t_z);

  float _restrict_thrust(std::array<float, NUM_THRUSTERS> & thrusts);
};

}  // namespace actuator_rp2040_driver

#endif  // _ACTUATOR_RP2040_DRIVER_HPP
