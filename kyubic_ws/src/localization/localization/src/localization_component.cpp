/**
 * @file localization_component.cpp
 * @brief localization using dvl , imu, and depth
 * @author R.Ohnishi
 * @date 2025/05/30
 *
 * @details DVL，IMU，Depthセンサを用いた自己位置推定
 ****************************************************/

#include "localization/localization_component.hpp"

#include "localization_msgs/msg/odometry.hpp"

#include <algorithm>

using namespace std::chrono_literals;

namespace localization
{

Localization::Localization(const rclcpp::NodeOptions & options) : Node("localization", options)
{
  odom_msg_ = std::make_shared<localization_msgs::msg::Odometry>();

  rclcpp::QoS qos(rclcpp::KeepLast(1));

  // Create publisher
  pub_ = create_publisher<localization_msgs::msg::Odometry>("odom", qos);

  // Create subscription
  sub_depth_ = create_subscription<localization_msgs::msg::Odometry>(
    "depth_odom", qos, std::bind(&Localization::_depth_callback, this, std::placeholders::_1));
  sub_imu_ = create_subscription<localization_msgs::msg::Odometry>(
    "imu_transformed", qos, std::bind(&Localization::_imu_callback, this, std::placeholders::_1));
  sub_depth_ = create_subscription<localization_msgs::msg::Odometry>(
    "dvl_odom", qos, std::bind(&Localization::_dvl_callback, this, std::placeholders::_1));

  // Create wall timer
  timer_ = create_wall_timer(10ms, std::bind(&Localization::_merge, this));
}

void Localization::_depth_callback(const localization_msgs::msg::Odometry::UniquePtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Updated Depth odometry");
  all_updated |= 4;

  odom_msg_->header = std::move(msg->header);
  odom_msg_->pose.position.z_depth = std::move(msg->pose.position.z_depth);
  odom_msg_->twist.linear.z_depth = std::move(msg->twist.linear.z_depth);
}

void Localization::_imu_callback(const localization_msgs::msg::Odometry::UniquePtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Updated IMU transformed");
  all_updated |= 2;

  odom_msg_->header = std::move(msg->header);
  odom_msg_->pose.orientation = std::move(msg->pose.orientation);
  odom_msg_->twist.angular = std::move(msg->twist.angular);
}

void Localization::_dvl_callback(const localization_msgs::msg::Odometry::UniquePtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Updated DVL odometry");
  all_updated |= 1;

  this->odom_msg_->header = std::move(msg->header);

  this->odom_msg_->pose.position.x = std::move(msg->pose.position.x);
  this->odom_msg_->pose.position.y = std::move(msg->pose.position.y);
  this->odom_msg_->pose.position.z_altitude = std::move(msg->pose.position.z_altitude);

  this->odom_msg_->pose.orientation = std::move(msg->pose.orientation);
  this->odom_msg_->twist.angular = std::move(msg->twist.angular);

  this->odom_msg_->twist.linear.x = std::move(msg->twist.linear.x);
  this->odom_msg_->twist.linear.y = std::move(msg->twist.linear.y);
  this->odom_msg_->twist.linear.z_altitude = std::move(msg->twist.linear.z_altitude);
}

void Localization::_merge()
{
  auto msg = std::make_unique<localization_msgs::msg::Odometry>();

  if (all_updated != 255) return;

  RCLCPP_INFO(this->get_logger(), "Updated localization");

  msg->header = odom_msg_->header;
  msg->status = odom_msg_->status;
  msg->pose = odom_msg_->pose;
  msg->twist = odom_msg_->twist;

  all_updated = 0b11111000;
  pub_->publish(std::move(msg));
}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::Localization)
