/**
 * @file dvl_odometry.hpp
 * @brief Calculating position by accumulating DVL velocity
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details DVLで取得した速度を累積して，位置を算出
 *********************************************************/

#include <localization/dvl_odometry_component.hpp>

#include <cmath>

namespace localization
{

DVLOdometry::DVLOdometry(const rclcpp::NodeOptions & options) : Node("dvl_odometry", options)
{
  imu_msg_ = std::make_shared<localization_msgs::msg::Odometry>();

  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<localization_msgs::msg::Odometry>("dvl_odom", qos);
  sub_imu_ = create_subscription<localization_msgs::msg::Odometry>(
    "imu_transformed", qos,
    std::bind(&DVLOdometry::_update_imu_callback, this, std::placeholders::_1));
  sub_dvl_ = create_subscription<driver_msgs::msg::DVL>(
    "dvl", qos, std::bind(&DVLOdometry::_update_dvl_callback, this, std::placeholders::_1));

  this->reset();
}

void DVLOdometry::_update_dvl_callback(const driver_msgs::msg::DVL::UniquePtr msg)
{
  // return if velocity error, otherwise calculate odometry
  if (msg->velocity_error == 32768) {
    RCLCPP_ERROR(this->get_logger(), "Don't calculate odometry. Because velocity error occurred");
    return;
  }

  // Calculate update period (delta t)
  auto now = this->get_clock()->now();
  double dt = (now - pre_time).nanoseconds() * 1e-9;
  pre_time = now;

  // Calculate angle considering offset
  double roll = imu_msg_->pose.orientation.x + offset_angle.at(0);
  double pitch = imu_msg_->pose.orientation.y + offset_angle.at(1);
  double heading = imu_msg_->pose.orientation.z + offset_angle.at(2);

  // Convert to warld coordinate system (right-handed coordinate system, and z-axis downward)
  // DVL coordinate system is right-handed cooordinate system, and z-axis upward.
  double vel_x = msg->velocity.y * cos(heading) - msg->velocity.x * sin(heading);
  double vel_y = msg->velocity.y * sin(heading) + msg->velocity.x * cos(heading);

  // Integral of velocity
  pos_x += vel_x * dt;
  pos_y += vel_y * dt;

  // Publish
  {
    auto odom_msg = std::make_unique<localization_msgs::msg::Odometry>();

    // Copy msg
    odom_msg->header = msg->header;
    odom_msg->twist.angular = imu_msg_->twist.angular;  // angle velocity

    // Add position
    odom_msg->pose.position.x = pos_x;
    odom_msg->pose.position.y = pos_y;
    odom_msg->pose.position.z_altitude = msg->altitude;

    // Add orientation
    odom_msg->pose.orientation.x = roll;
    odom_msg->pose.orientation.y = pitch;
    odom_msg->pose.orientation.z = heading;

    // Add linear velocity
    odom_msg->twist.linear.x = vel_x;
    odom_msg->twist.linear.y = vel_y;
    odom_msg->twist.linear.z_altitude = msg->velocity.z;

    pub_->publish(std::move(odom_msg));
  }
  RCLCPP_INFO(this->get_logger(), "Calculated DVL odometry");
}

void DVLOdometry::_update_imu_callback(localization_msgs::msg::Odometry::UniquePtr msg)
{
  imu_msg_ = std::move(msg);
}

void DVLOdometry::reset(const std::array<double, 3> offset_angle)
{
  pre_time = this->get_clock()->now();
  this->offset_angle = offset_angle;
  pos_x = pos_y = 0.0;
}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::DVLOdometry)
