/**
 * @file depth_velocity.cpp
 * @brief Calculating velocity by differentiating depth data
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details 深度センサのデータを微分してz-axisの速度を算出
 **********************************************************/

#include "localization/depth_velocity.hpp"

#include "driver_msgs/msg/depth.hpp"

#include <array>
#include <cstdint>
#include <functional>

namespace localization
{

DepthVelocity::DepthVelocity() : Node("depth_velocity")
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  sub_ = create_subscription<driver_msgs::msg::Depth>(
    "driver/depth", qos, std::bind(&DepthVelocity::_update_callback, this, std::placeholders::_1));

  this->reset();
}

void DepthVelocity::_update_callback(const driver_msgs::msg::Depth::UniquePtr msg)
{
  // TODO: mgsの有効性チェック
  auto now = this->get_clock()->now();
  uint64_t dt = (now - pre_time).nanoseconds();
  pre_time = now;

  pos_z = msg->depth;
  vel_z = (pos_z - pre_pos_z) / dt;
}

std::array<double, 2> DepthVelocity::get_depth()
{
  std::array<double, 2> depth_info = {vel_z, pos_z};
  return depth_info;
}

void DepthVelocity::reset()
{
  pre_pos_z = 0.0;
  pre_time = this->get_clock()->now();
}

}  // namespace localization
