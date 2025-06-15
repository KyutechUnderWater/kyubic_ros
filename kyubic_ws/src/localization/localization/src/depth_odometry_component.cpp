/**
 * @file depth_odometry.cpp
 * @brief Calculating velocity by differentiating depth data
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details 深度センサのデータを微分してz-axisの速度を算出
 **********************************************************/

#include "localization/depth_odometry_component.hpp"

namespace localization
{

DepthOdometry::DepthOdometry(const rclcpp::NodeOptions & options) : Node("depth_odometry", options)
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<localization_msgs::msg::Odometry>("depth_odom", qos);
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
  double dt = (now - pre_time).nanoseconds() * 1e-9;
  pre_time = now;

  // calculate moving avelage
  pos_z_list.at(idx) = msg->depth;
  double pos_z_sum = 0.0;
  for (u_int8_t i = 0; i < pos_z_list.size(); i++) {
    pos_z_sum += pos_z_list.at(i);
  }
  double pos_z = pos_z_sum / pos_z_list.size();

  // calculate depth velocity
  std::cout << pos_z << " " << pre_pos_z << " " << dt << std::endl;
  double vel_z = (pos_z - pre_pos_z) / dt;
  pre_pos_z = pos_z;

  // Publish
  {
    auto odom_msg = std::make_unique<localization_msgs::msg::Odometry>();

    // copy msg
    odom_msg->header = msg->header;
    odom_msg->pose.position.z_depth = msg->depth;

    // add velocity
    odom_msg->twist.linear.z_depth = vel_z;

    pub_->publish(std::move(odom_msg));
  }
  RCLCPP_INFO(this->get_logger(), "Calculated depth odometry");
}

void DepthOdometry::reset()
{
  pre_pos_z = 0.0;
  pre_time = this->get_clock()->now();
}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::DepthOdometry)
