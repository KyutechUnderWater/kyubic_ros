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
#include <numbers>

namespace localization
{

IMUTransform::IMUTransform(const rclcpp::NodeOptions & options) : Node("imu_transform", options)
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<localization_msgs::msg::Odometry>("imu_transformed", qos);
  sub_ = create_subscription<driver_msgs::msg::IMU>(
    "imu", qos, std::bind(&IMUTransform::_transform_callback, this, std::placeholders::_1));
}

void IMUTransform::_transform_callback(const driver_msgs::msg::IMU::UniquePtr msg)
{
  const double sin180 = sin(std::numbers::pi);
  const double cos180 = cos(std::numbers::pi);

  // z-axis transform
  double gyro_x = msg->gyro.x * cos180 - msg->gyro.y * sin180;
  double gyro_y = msg->gyro.x * sin180 + msg->gyro.y * cos180;

  double roll = msg->orient.x * cos180 - msg->orient.y * sin180;
  double pitch = msg->orient.x * sin180 + msg->orient.y * cos180;

  // Publish
  {
    auto odom_msg = std::make_unique<localization_msgs::msg::Odometry>();

    odom_msg->twist.angular.x = gyro_x;
    odom_msg->twist.angular.y = gyro_y;
    odom_msg->twist.angular.z = msg->gyro.z;

    odom_msg->pose.orientation.x = roll;
    odom_msg->pose.orientation.y = pitch;
    odom_msg->pose.orientation.z = msg->orient.z;

    pub_->publish(std::move(odom_msg));
  }
  RCLCPP_INFO(this->get_logger(), "Calculated IMU transform");
}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::IMUTransform)
