/**
 * @file thruster_driver.hpp
 * @brief thruster driver
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details 各軸の推力をサブスクライブし，ESC制御マイコンに命令する
 *******************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.hpp>
#include <timer/timeout.hpp>

#include "std_msgs/msg/bool.hpp"
#include <driver_msgs/msg/thruster.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <array>
#include <cmath>

/**
 * @namespace thruster_driver
 * @brief For thruster driver
 */
namespace thruster_driver
{

constexpr float theta_h = 30 * M_PI / 180;  //[rad]
constexpr float theta_v = 30 * M_PI / 180;  //[rad]

// Distance from center of robot to center of thruster
const float dist_hx = 0.19;  /// x-axis (horizontal)
const float dist_hy = 0.14;  /// y-axis (horizontal)
// TODO: 垂直方向スラスタの位置
const float dist_vy = 0.19;  /// x-axis (vertical)
const float dist_vz = 0.14;  /// y-axis (vertical)
const float theta_dist_h = atan(dist_hy / dist_hx);
const float theta_dist_v = atan(dist_vy / dist_vz);

// Force to Thrust, Torque to Thrust scale
const float f_x_scale = 1 / (4 * cos(theta_h));
const float f_y_scale = 1 / (4 * sin(theta_h));
const float f_z_scale = 1 / (2 * cos(theta_v));
const float t_x_scale =
  1 / (2 * sqrt(pow(dist_vy, 2) + pow(dist_vz, 2)) * cos(theta_h - theta_dist_v));
const float t_z_scale =
  1 / (4 * sqrt(pow(dist_hx, 2) + pow(dist_hy, 2)) * cos(theta_h - theta_dist_h));

const uint8_t NUM_THRUSTERS = 6;

/**
 * @brief Thruster class
 */
class ThrusterDriver : public rclcpp::Node
{
public:
  /**
   * @brief Serial and ROS-topic setting
   * @details Serial setting and Define a subscriber
   */
  explicit ThrusterDriver();

private:
  std::string portname;
  int baudrate;
  float max_thrust;
  float max_thrust_per;
  uint64_t timeout;

  const char start_char = {'*'};
  const char delim_char = {';'};
  const char end_char = {'#'};

  bool heartbeat = false;

  std::shared_ptr<serial::Serial> serial_;
  rclcpp::Publisher<driver_msgs::msg::Thruster>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_heartbeat_;

  std::shared_ptr<timer::Timeout> timeout_;

  std::array<float, NUM_THRUSTERS> _wrench2thrusts(
    float f_x, float f_y, float f_z, float t_x, float t_z);
  float _restrict_thrust(std::array<float, NUM_THRUSTERS> thrusts);
  void robot_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void heartbeat_callback(const std_msgs::msg::Bool::SharedPtr msg);
};

}  // namespace thruster_driver
