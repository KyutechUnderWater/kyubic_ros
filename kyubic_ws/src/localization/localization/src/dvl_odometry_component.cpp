/**
 * @file dvl_odometry.hpp
 * @brief Calculating position by accumulating DVL velocity
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details DVLで取得した速度を累積して，位置を算出
 *********************************************************/

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <localization/dvl_odometry_component.hpp>

using namespace std::chrono_literals;

namespace localization
{

DVLOdometry::DVLOdometry(const rclcpp::NodeOptions & options) : Node("dvl_odometry", options)
{
  offset[0] = this->declare_parameter("offset_x", 0.0);
  offset[1] = this->declare_parameter("offset_y", 0.0);
  offset[2] = this->declare_parameter("offset_z", 0.0);

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
  if (!msg->velocity_valid) {
    RCLCPP_ERROR(this->get_logger(), "Don't calculate odometry. Because velocity error occurred");
    odom_msg->header = msg->header;
    odom_msg->status.dvl = localization_msgs::msg::Status::ERROR;
  } else {
    // Calculate update period (delta t)
    auto now = this->get_clock()->now();
    double dt = (now - pre_time).nanoseconds() * 1e-9;
    pre_time = now;

    // Define Offset Vector (Lever Arm) from Center of Rotation to DVL
    tf2::Vector3 lever_arm(offset[0], offset[1], offset[2]);
    tf2::Vector3 vel_raw(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    tf2::Vector3 alt_vec_raw(0.0, 0.0, msg->altitude);
    tf2::Vector3 angular_vel(
      imu_msg_->twist.angular.x * RADIAN_SCALE, imu_msg_->twist.angular.y * RADIAN_SCALE,
      imu_msg_->twist.angular.z * RADIAN_SCALE);

    // Create Rotation Quaternion
    tf2::Quaternion q_rot;
    q_rot.setRPY(
      imu_msg_->pose.orientation.x * RADIAN_SCALE, imu_msg_->pose.orientation.y * RADIAN_SCALE,
      imu_msg_->pose.orientation.z * RADIAN_SCALE);

    // Transform Body to World Coordinate System (right-handed coordinate system, and z-axis downward)
    tf2::Vector3 vel_raw_world = tf2::quatRotate(q_rot, vel_raw);
    tf2::Vector3 angular_vel_world = tf2::quatRotate(q_rot, angular_vel);
    tf2::Vector3 lever_arm_world = tf2::quatRotate(q_rot, lever_arm);
    tf2::Vector3 alt_vec_world = tf2::quatRotate(q_rot, alt_vec_raw);

    // Integral of velocity
    pos_x += vel_raw_world.x() * dt;
    pos_y += vel_raw_world.y() * dt;

    // Apply Lever Arm Correction
    tf2::Vector3 vel_robot_world = vel_raw_world - angular_vel_world.cross(lever_arm_world);
    double pos_robot_world_x = pos_x - lever_arm_world.x() + offset[0];
    double pos_robot_world_y = pos_y - lever_arm_world.y() + offset[1];
    double pos_robot_world_z = alt_vec_world.z() - lever_arm_world.z() + offset[2];

    // Publish
    {
      // Copy msg
      odom_msg->header = msg->header;

      // Add position
      odom_msg->pose.position.x = pos_robot_world_x;
      odom_msg->pose.position.y = pos_robot_world_y;
      odom_msg->pose.position.z_altitude = pos_robot_world_z;

      // Add orientation
      odom_msg->pose.orientation = imu_msg_->pose.orientation;
      odom_msg->twist.angular = imu_msg_->twist.angular;

      // Add linear velocity
      odom_msg->twist.linear.x = vel_robot_world.x();
      odom_msg->twist.linear.y = vel_robot_world.y();
      odom_msg->twist.linear.z_altitude = vel_robot_world.z();
    }
    RCLCPP_DEBUG(this->get_logger(), "Calculated DVL odometry");
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
