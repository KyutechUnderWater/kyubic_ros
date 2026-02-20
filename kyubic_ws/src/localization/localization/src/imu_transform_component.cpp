/**
 * @file imu_transform_component.cpp
 * @brief convert from imu frame to map frame
 * @author R.Ohnishi
 * @date 2025/05/30
 *
 * @details imuの座標系をワールド座標系に変換する
 ***********************************************/

#include "localization/imu_transform_component.hpp"

namespace localization::imu
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
  auto odom_msg = std::make_unique<localization_msgs::msg::Odometry>();

  if (msg->status.id == common_msgs::msg::Status::ERROR) {
    RCLCPP_ERROR(this->get_logger(), "The imu data is invalid");
    odom_msg->header = msg->header;
    odom_msg->status.imu.id = common_msgs::msg::Status::ERROR;
  } else {
    // z-axis transform
    double accel_x = msg->accel.y;
    double accel_y = -msg->accel.y;
    double gyro_x = msg->gyro.y;
    double gyro_y = -msg->gyro.x;

    // circular buffer for calculate offset
    accel_z[count++ % 100] = msg->accel.z;
    if (count == 1e6) count = 100;

    roll = msg->orient.y;
    pitch = -msg->orient.x;
    yaw = msg->orient.z;

    double yaw_offset = msg->orient.z - offset_angle.at(2);
    if (yaw_offset < -180) yaw_offset += 360;
    if (180 < yaw_offset) yaw_offset -= 360;

    // Publish
    {
      odom_msg->header = msg->header;
      odom_msg->status.imu.id = common_msgs::msg::Status::NORMAL;

      odom_msg->accel.linear.x = accel_x;
      odom_msg->accel.linear.y = accel_y;
      odom_msg->accel.linear.z_depth = msg->accel.z - offset_accel_z;
      odom_msg->accel.linear.z_altitude = -msg->accel.z + offset_accel_z;

      odom_msg->twist.angular.x = gyro_x;
      odom_msg->twist.angular.y = gyro_y;
      odom_msg->twist.angular.z = msg->gyro.z;

      odom_msg->pose.orientation.x = roll - offset_angle.at(0);
      odom_msg->pose.orientation.y = pitch - offset_angle.at(1);
      odom_msg->pose.orientation.z = yaw_offset;
    }
    RCLCPP_DEBUG(this->get_logger(), "Calculated IMU transform");
  }
  pub_->publish(std::move(odom_msg));
}

void IMUTransform::reset_callback(
  [[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr request,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  this->reset();
  RCLCPP_INFO(this->get_logger(), "Reset");

  response->success = true;
  response->message = "IMU transform reset successfully";
}

void IMUTransform::reset()
{
  // NOTE: After 2seconds(when 50Hz) from node start, available to calculate offset
  offset_angle = {roll, pitch, yaw};
  offset_accel_z = std::accumulate(accel_z.begin(), accel_z.end(), 0.0) / accel_z.size();
}

}  // namespace localization::imu

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::imu::IMUTransform)
