/**
 * @file depth_velocity.hpp
 * @brief Calculating velocity by differentiating depth data
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details 深度センサのデータを微分してz-axisの速度を算出
 **********************************************************/

#ifndef _DEPTH_VELOCITY_HPP
#define _DEPTH_VELOCITY_HPP

#include <rclcpp/rclcpp.hpp>

#include <driver_msgs/msg/depth.hpp>

#include <array>

namespace depth_driver
{

class DepthOdometry : public rclcpp::Node
{
private:
  rclcpp::Publisher<driver_msgs::msg::Depth>::SharedPtr pub_;
  rclcpp::Subscription<driver_msgs::msg::Depth>::SharedPtr sub_;
  rclcpp::Time pre_time;

  double pre_pos_z = 0.0;
  double pos_z = 0.0;
  double vel_z = 0.0;

public:
  explicit DepthOdometry(const rclcpp::NodeOptions & options);

  void _update_callback(const driver_msgs::msg::Depth::UniquePtr msg);

  std::array<double, 2> get_depth();

  void reset();
};

}  // namespace depth_driver

#endif  // !_DEPTH_VELOCITY_HPP
