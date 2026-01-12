/**
 * @file depth_odometry.cpp
 * @brief Calculating velocity by differentiating depth data
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details 深度センサのデータを微分してz-axisの速度を算出
 **********************************************************/

#include "localization/depth_odometry_component.hpp"

#include <functional>
#include <numeric>

#include "localization_msgs/msg/status.hpp"

namespace localization
{

DepthOdometry::DepthOdometry(const rclcpp::NodeOptions & options) : Node("depth_odometry", options)
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<localization_msgs::msg::Odometry>("odom", qos);
  sub_ = create_subscription<driver_msgs::msg::Depth>(
    "depth", qos, std::bind(&DepthOdometry::update_callback, this, std::placeholders::_1));
  srv_ = create_service<std_srvs::srv::Trigger>(
    "reset",
    std::bind(&DepthOdometry::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

  this->reset();
}

void DepthOdometry::update_callback(const driver_msgs::msg::Depth::UniquePtr msg)
{
  auto odom_msg = std::make_unique<localization_msgs::msg::Odometry>();

  // return if msg is invalid, otherwaise calculate depth velocity
  if (msg->status.id == common_msgs::msg::Status::ERROR) {
    RCLCPP_ERROR(this->get_logger(), "The depth data is invalid");
    odom_msg->header = msg->header;
    odom_msg->status.depth = localization_msgs::msg::Status::ERROR;
  } else {
    // calculate period (delta t)
    auto now = this->get_clock()->now();
    double dt = (now - pre_time).nanoseconds() * 1e-9;
    pre_time = now;

    // calculate exponential moving average
    pos_z = EMA_ALPHA * msg->depth + (1.0 - EMA_ALPHA) * pre_pos_z;

    // calculate depth velocity
    double vel_z = (pos_z - pre_pos_z) / dt;

    // calculate moving avelage
    vel_z_list.at(idx) = vel_z;
    double vel_z_sum = std::accumulate(vel_z_list.begin(), vel_z_list.end(), 0.0);
    vel_z = vel_z_sum / static_cast<double>(vel_z_list.size());

    // prepare for next step
    pre_pos_z = pos_z;
    if (++idx == vel_z_list.size()) idx = 0;

    // Publish
    {
      // copy msg
      odom_msg->header = msg->header;
      odom_msg->pose.position.z_depth = pos_z - offset_pos_z;

      // add velocity
      odom_msg->twist.linear.z_depth = vel_z;
    }
    RCLCPP_DEBUG(this->get_logger(), "Calculated depth odometry");
  }
  pub_->publish(std::move(odom_msg));
}

void DepthOdometry::reset_callback(
  [[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr request,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  this->reset();
  RCLCPP_INFO(this->get_logger(), "Depth odometry reset");

  response->success = true;
  response->message = "";
}

void DepthOdometry::reset()
{
  offset_pos_z = pos_z;
  pre_pos_z = 0.0;
  pre_time = this->get_clock()->now();
}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::DepthOdometry)
