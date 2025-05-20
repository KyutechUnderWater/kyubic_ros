/**
 * @file depth_velocity.cpp
 * @brief Calculating velocity by differentiating depth data
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details 深度センサのデータを微分してz-axisの速度を算出
 **********************************************************/

#include "depth_driver/depth_odometry_component.hpp"

#include "driver_msgs/msg/depth.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>

namespace depth_driver
{

DepthOdometry::DepthOdometry(const rclcpp::NodeOptions & options) : Node("depth_odometry", options)
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  pub_ = create_publisher<driver_msgs::msg::Depth>("depth_odometry", qos);
  sub_ = create_subscription<driver_msgs::msg::Depth>(
    "depth", qos, std::bind(&DepthOdometry::_update_callback, this, std::placeholders::_1));

  this->reset();
}

void DepthOdometry::_update_callback(const driver_msgs::msg::Depth::UniquePtr msg)
{
  // return if msg is disable, otherwaise calculate depth velocity
  // TODO:

  // calculate period (delta t)
  auto now = this->get_clock()->now();
  uint64_t dt = (now - pre_time).nanoseconds();
  pre_time = now;

  // calculate depth velocity
  pos_z = msg->pos_z;
  vel_z = (pos_z - pre_pos_z) / dt;
  pre_pos_z = pos_z;

  // Publish
  {
    auto vel_msg = std::make_unique<driver_msgs::msg::Depth>();

    // copy msg
    vel_msg->header = msg->header;
    vel_msg->pos_z = msg->pos_z;

    // add velocity
    vel_msg->vel_z = vel_z;
  }
}

std::array<double, 2> DepthOdometry::get_depth()
{
  std::array<double, 2> depth_info = {vel_z, pos_z};
  return depth_info;
}

void DepthOdometry::reset()
{
  pre_pos_z = 0.0;
  pre_time = this->get_clock()->now();
}

}  // namespace depth_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depth_driver::DepthOdometry)
