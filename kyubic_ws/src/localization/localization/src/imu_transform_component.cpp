/**
 * @file imu_transform_component.cpp
 * @brief convert from imu frame to map frame
 * @author R.Ohnishi
 * @date 2025/05/30
 *
 * @details imuの座標系をワールド座標系に変換する
 ***********************************************/

#include "localization/imu_transform_component.hpp"

#include "driver_msgs/msg/imu.hpp"

#include <cmath>
#include <functional>
#include <numbers>

namespace localization
{

IMUTransform::IMUTransform(const rclcpp::NodeOptions & options) : Node("imu_transform", options)
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<localization_msgs::msg::Odometry>("transformed", qos);
  sub_ = create_subscription<driver_msgs::msg::IMU>(
    "imu", qos, std::bind(&IMUTransform::update_callback, this, std::placeholders::_1));
  srv_ = create_service<std_srvs::srv::Trigger>(
    "reset",
    std::bind(&IMUTransform::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

  reset();
}

void IMUTransform::update_callback(const driver_msgs::msg::IMU::UniquePtr msg)
{
  // define
  const double sin180 = sin(std::numbers::pi);
  const double cos180 = cos(std::numbers::pi);

  // z-axis transform
  double gyro_x = msg->gyro.x * cos180 - msg->gyro.y * sin180;
  double gyro_y = msg->gyro.x * sin180 + msg->gyro.y * cos180;

  roll = msg->orient.x * cos180 - msg->orient.y * sin180;
  pitch = msg->orient.x * sin180 + msg->orient.y * cos180;
  yaw = msg->orient.z;

  // Publish
  {
    auto odom_msg = std::make_unique<localization_msgs::msg::Odometry>();

    odom_msg->twist.angular.x = gyro_x;
    odom_msg->twist.angular.y = gyro_y;
    odom_msg->twist.angular.z = msg->gyro.z;

    odom_msg->pose.orientation.x = roll - offset_angle.at(0);
    odom_msg->pose.orientation.y = pitch - offset_angle.at(1);
    odom_msg->pose.orientation.z = yaw - offset_angle.at(2);

    pub_->publish(std::move(odom_msg));
  }
  RCLCPP_INFO(this->get_logger(), "Calculated IMU transform");
}

void IMUTransform::reset_callback(
  [[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr request,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  this->reset();
  RCLCPP_INFO(this->get_logger(), "Reset");

  response->success = true;
  response->message = "";
}

void IMUTransform::reset() { offset_angle = {roll, pitch, yaw}; }

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::IMUTransform)
