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

namespace localization
{

class DepthVelocity : rclcpp::Node
{
private:
  rclcpp::Subscription<driver_msgs::msg::Depth>::SharedPtr sub_;
  rclcpp::Time pre_time;

  double pre_pos_z = 0.0;
  double pos_z = 0.0;
  double vel_z = 0.0;

public:
  explicit DepthVelocity();

  void _update_callback(const driver_msgs::msg::Depth::UniquePtr msg);

  std::array<double, 2> get_depth();

  void reset();
};

}  // namespace localization

#endif  // !_DEPTH_VELOCITY_HPP
