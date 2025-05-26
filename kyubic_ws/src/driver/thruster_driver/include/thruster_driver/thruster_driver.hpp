/**
 * @file thruster_driver.hpp
 * @brief thruster driver
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details 各軸の推力をサブスクライブし，ESC制御マイコンに命令する
 *******************************************************************/

#include <cmath>
#include <driver_msgs/msg/thruster.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.hpp>

/**
 * @namespace thruster_driver
 * @brief For thruster driver
 */
namespace thruster_driver
{

// TODO: doxygen commentの追加
constexpr uint theta_h = 30;  //[deg]
constexpr uint theta_v = 30;  //[deg]
const float sin_h = sin(theta_h * M_PI / 180);
const float cos_h = cos(theta_h * M_PI / 180);
const float sin_v = sin(theta_v * M_PI / 180);

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

  const char start_char = {'*'};
  const char delim_char = {';'};
  const char end_char = {'#'};

  /// 最大推力値 [N]
  unsigned int max_thrust = 60;

  std::shared_ptr<serial::Serial> serial_;
  rclcpp::Publisher<driver_msgs::msg::Thruster>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;

  float _calc_restrict_rate(const float total, const float limit);
  void _robot_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
};

}  // namespace thruster_driver
