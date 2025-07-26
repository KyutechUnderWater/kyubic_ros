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
#include <functional>
#include <numbers>

using namespace std::chrono_literals;

namespace localization
{

DVLOdometry::DVLOdometry(const rclcpp::NodeOptions & options) : Node("dvl_odometry", options)
{
  imu_msg_ = std::make_shared<localization_msgs::msg::Odometry>();

  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<localization_msgs::msg::Odometry>("odom", qos);
  sub_imu_ = create_subscription<localization_msgs::msg::Odometry>(
    "transformed_imu", qos,
    std::bind(&DVLOdometry::update_imu_callback, this, std::placeholders::_1));
  sub_dvl_ = create_subscription<driver_msgs::msg::DVL>(
    "dvl", qos, std::bind(&DVLOdometry::update_callback, this, std::placeholders::_1));
  srv_ = create_service<std_srvs::srv::Trigger>(
    "reset",
    std::bind(&DVLOdometry::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

  this->reset();
}

void DVLOdometry::update_callback(const driver_msgs::msg::DVL::UniquePtr msg)
{
  auto odom_msg = std::make_unique<localization_msgs::msg::Odometry>();

  // return if velocity error, otherwise calculate odometry
  if (msg->velocity_error == -32768) {
    RCLCPP_ERROR(this->get_logger(), "Don't calculate odometry. Because velocity error occurred");
    odom_msg->status.dvl = localization_msgs::msg::Status::ERROR;
  } else {
    // Calculate update period (delta t)
    auto now = this->get_clock()->now();
    double dt = (now - pre_time).nanoseconds() * 1e-9;
    pre_time = now;

    // Convert to warld coordinate system (right-handed coordinate system, and z-axis downward)
    // DVL coordinate system is right-handed cooordinate system, and z-axis upward.
    double heading_rad = imu_msg_->pose.orientation.z * std::numbers::pi / 180;
    double vel_x = msg->velocity.x * cos(heading_rad) - msg->velocity.y * sin(heading_rad);
    double vel_y = msg->velocity.x * sin(heading_rad) + msg->velocity.y * cos(heading_rad);

    // Integral of velocity
    pos_x += vel_x * dt;
    pos_y += vel_y * dt;

    // Publish
    {
      // Copy msg
      odom_msg->header = msg->header;
      odom_msg->twist.angular = imu_msg_->twist.angular;  // angle velocity

      // Add position
      odom_msg->pose.position.x = pos_x;
      odom_msg->pose.position.y = pos_y;
      odom_msg->pose.position.z_altitude = msg->altitude;

      // Add orientation
      odom_msg->pose.orientation = imu_msg_->pose.orientation;

      // Add linear velocity
      odom_msg->twist.linear.x = vel_x;
      odom_msg->twist.linear.y = vel_y;
      odom_msg->twist.linear.z_altitude = msg->velocity.z;
    }
    RCLCPP_INFO(this->get_logger(), "Calculated DVL odometry");
  }
  pub_->publish(std::move(odom_msg));
}

void DVLOdometry::update_imu_callback(localization_msgs::msg::Odometry::UniquePtr msg)
{
  imu_msg_ = std::move(msg);
}

void DVLOdometry::reset_callback(
  [[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr request,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  this->reset();
  RCLCPP_INFO(this->get_logger(), "Reset");

  response->success = true;
  response->message = "";
}

void DVLOdometry::reset()
{
  pre_time = this->get_clock()->now();
  pos_x = pos_y = 0.0;
}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::DVLOdometry)
