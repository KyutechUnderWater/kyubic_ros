/**
 * @file test_pid.cpp
 * @brief Test code for PID Controller
 * @author R.Ohnishi
 * @date 2025/05/14
 *
 * @details PID制御のテストコード
 **************************************************/

#include "test_pid/test_pid.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"

#include <iostream>

using namespace std::chrono_literals;

namespace test
{

TestPID::TestPID(const rclcpp::NodeOptions & options) : Node("test_pid", options)
{
  _declare_parameter();

  for (uint8_t i = 0; i < name.size(); i++) {
    vp_pids[name.at(i)] = std::make_shared<pid_controller::VelocityP_PID>(p_pid_params[name.at(i)]);
  }

  odom_ = std::make_shared<localization_msgs::msg::Odometry>();
  joy_ = std::make_shared<geometry_msgs::msg::WrenchStamped>();
  targets_ = std::make_shared<test_pid_msgs::msg::Targets>();

  // ROS 2 communication
  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("robot_force", qos);
  sub_target_ = create_subscription<test_pid_msgs::msg::Targets>(
    "target", qos, std::bind(&TestPID::callback_target, this, std::placeholders::_1));
  sub_joy_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    "robot_force", qos, std::bind(&TestPID::callback_joy, this, std::placeholders::_1));
  sub_odom_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&TestPID::callback_odom, this, std::placeholders::_1));
  timer_ = create_wall_timer(10ms, std::bind(&TestPID::update, this));
}

void TestPID::_declare_parameter()
{
  // Set parameters
  for (uint8_t i = 0; i < name.size(); i++) {
    p_pid_params[name.at(i)] = std::make_shared<pid_controller::VelocityP_PIDParameter>();
    p_pid_params[name.at(i)]->k = this->declare_parameter(name.at(i) + ".k", 0.0);
    p_pid_params[name.at(i)]->lo = this->declare_parameter(name.at(i) + ".lo", 0.0);
    p_pid_params[name.at(i)]->hi = this->declare_parameter(name.at(i) + ".hi", 0.0);
    p_pid_params[name.at(i)]->vpid_param.kp = this->declare_parameter(name.at(i) + ".vpid.kp", 0.0);
    p_pid_params[name.at(i)]->vpid_param.ki = this->declare_parameter(name.at(i) + ".vpid.ki", 0.0);
    p_pid_params[name.at(i)]->vpid_param.kd = this->declare_parameter(name.at(i) + ".vpid.kd", 0.0);
    p_pid_params[name.at(i)]->vpid_param.kf = this->declare_parameter(name.at(i) + ".vpid.kf", 0.0);
    p_pid_params[name.at(i)]->vpid_param.lo = this->declare_parameter(name.at(i) + ".vpid.lo", 0.0);
    p_pid_params[name.at(i)]->vpid_param.hi = this->declare_parameter(name.at(i) + ".vpid.hi", 0.0);
  }
}

void TestPID::update()
{
  if (updated) {
    updated = false;

    double p_pid_x, p_pid_y, p_pid_z, p_pid_yaw;
    p_pid_x = p_pid_y = p_pid_z = p_pid_yaw = 0.0;
    // x-axis
    p_pid_x =
      vp_pids[name.at(0)]->update(odom_->twist.linear.x, odom_->pose.position.x, targets_->x);

    // y-axis
    p_pid_y =
      vp_pids[name.at(1)]->update(odom_->twist.linear.y, odom_->pose.position.y, targets_->y);

    // z-axis
    // vpid_z = vpid_->update(odom_->twist.linear.z_depth, target);
    p_pid_z = -vp_pids[name.at(2)]->update(
      odom_->twist.linear.z_altitude, odom_->pose.position.z_altitude, targets_->z);

    // yaw-axis
    double target_yaw = targets_->yaw;
    if (targets_->yaw - odom_->pose.orientation.z < 180) target_yaw += 360;
    if (target_yaw - odom_->pose.orientation.z > 180) target_yaw -= 360;
    p_pid_yaw =
      vp_pids[name.at(3)]->update(odom_->twist.angular.z, odom_->pose.orientation.z, target_yaw);

    std::cout << "target: " << target_yaw << " current: " << odom_->twist.angular.z
              << " current_t: " << odom_->pose.orientation.z << " p_pid: " << p_pid_yaw
              << std::endl;

    // Create message from pid
    auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
    // msg->wrench.force.x = p_pid_x;
    // msg->wrench.force.y = p_pid_y;
    msg->wrench.force.x = joy_->wrench.force.x;
    msg->wrench.force.y = joy_->wrench.force.y;
    msg->wrench.force.z = p_pid_z;
    msg->wrench.torque.z = p_pid_yaw;

    pub_->publish(std::move(msg));
  }
}

void TestPID::callback_target(test_pid_msgs::msg::Targets::UniquePtr msg)
{
  targets_ = std::move(msg);
}

void TestPID::callback_joy(geometry_msgs::msg::WrenchStamped::UniquePtr msg)
{
  joy_ = std::move(msg);
}

void TestPID::callback_odom(localization_msgs::msg::Odometry::UniquePtr msg)
{
  updated = true;
  odom_ = std::move(msg);
}

}  // namespace test

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(test::TestPID)
