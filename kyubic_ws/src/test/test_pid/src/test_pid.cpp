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
  // Declere parameters
  _declare_parameter();

  // Create PID controller instance
  for (uint8_t i = 0; i < name.size(); i++) {
    vp_pids[name.at(i)] = std::make_shared<pid_controller::VelocityP_PID>(p_pid_params[name.at(i)]);
  }

  // Create messages instance
  odom_ = std::make_shared<localization_msgs::msg::Odometry>();
  joy_ = std::make_shared<geometry_msgs::msg::WrenchStamped>();
  targets_ = std::make_shared<rt_pose_plotter_msgs::msg::Targets>();

  // ROS 2 communication
  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("robot_force", qos);
  sub_targets_ = create_subscription<rt_pose_plotter_msgs::msg::Targets>(
    "targets", qos, std::bind(&TestPID::callback_target, this, std::placeholders::_1));
  sub_joy_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    "joy_robot_force", qos, std::bind(&TestPID::callback_joy, this, std::placeholders::_1));
  sub_odom_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&TestPID::callback_odom, this, std::placeholders::_1));
  timer_ = create_wall_timer(10ms, std::bind(&TestPID::update, this));
}

void TestPID::_declare_parameter()
{
  // Set parameters
  for (uint8_t i = 0; i < name.size(); i++) {
    p_pid_params[name.at(i)] = pid_controller::VelocityP_PIDParameter();
    p_pid_params[name.at(i)].k = this->declare_parameter(name.at(i) + ".k", 0.0);
    p_pid_params[name.at(i)].lo = this->declare_parameter(name.at(i) + ".lo", 0.0);
    p_pid_params[name.at(i)].hi = this->declare_parameter(name.at(i) + ".hi", 0.0);
    p_pid_params[name.at(i)].vpid_param.kp = this->declare_parameter(name.at(i) + ".vpid.kp", 0.0);
    p_pid_params[name.at(i)].vpid_param.ki = this->declare_parameter(name.at(i) + ".vpid.ki", 0.0);
    p_pid_params[name.at(i)].vpid_param.kd = this->declare_parameter(name.at(i) + ".vpid.kd", 0.0);
    p_pid_params[name.at(i)].vpid_param.kf = this->declare_parameter(name.at(i) + ".vpid.kf", 0.0);
    p_pid_params[name.at(i)].vpid_param.lo = this->declare_parameter(name.at(i) + ".vpid.lo", 0.0);
    p_pid_params[name.at(i)].vpid_param.hi = this->declare_parameter(name.at(i) + ".vpid.hi", 0.0);
  }
  p_pid_params["z"].vpid_param.offset = this->declare_parameter("z.vpid.offset", 0.0);
}

void TestPID::update()
{
  if (updated) {
    updated = false;

    // Define valiable
    double p_pid_x, p_pid_y, p_pid_z, p_pid_roll, p_pid_yaw;
    p_pid_x = p_pid_y = p_pid_z = p_pid_yaw = 0.0;

    // Abbreviation of name
    auto pose = odom_->pose.position;
    auto orient = odom_->pose.orientation;
    auto linear = odom_->twist.linear;
    auto angular = odom_->twist.angular;

    // x-axis
    p_pid_x = vp_pids[name.at(0)]->update(linear.x, pose.x, targets_->pose.x);

    // y-axis
    p_pid_y = vp_pids[name.at(1)]->update(linear.y, pose.y, targets_->pose.y);

    // z-axis
    p_pid_z = vp_pids[name.at(2)]->update(linear.z_depth, pose.z_depth, targets_->pose.z_depth);
    // p_pid_z = -vp_pids[name.at(2)]->update(linear.z_altitude, pose.z_altitude, targets_->pose.z_altitude);

    // roll-axis
    p_pid_roll = vp_pids[name.at(3)]->update(angular.x, orient.x, targets_->pose.roll);

    // yaw-axis
    double target_yaw = targets_->pose.yaw;
    if (targets_->pose.yaw - orient.z < -180) target_yaw += 360;
    if (targets_->pose.yaw - orient.z > 180) target_yaw -= 360;
    p_pid_yaw = vp_pids[name.at(4)]->update(angular.z, orient.z, target_yaw);

    // z-axis transform
    double z_rad = -orient.z * std::numbers::pi / 180;
    double _p_pid_x = p_pid_x;
    double _p_pid_y = p_pid_y;
    p_pid_x = _p_pid_x * cos(z_rad) - _p_pid_y * sin(z_rad);
    p_pid_y = _p_pid_x * sin(z_rad) + _p_pid_y * cos(z_rad);

    // Print data
    double target_p = targets_->pose.z_depth;
    double current_p = pose.z_depth;
    double current_vel_p = linear.z_depth;
    double p_pid_p = p_pid_z;
    std::cout << "target: " << target_p << " current: " << current_p
              << " current_vel: " << current_vel_p << " p_pid: " << p_pid_p << std::endl;

    // Create message from pid
    auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
    msg->wrench.force.x = p_pid_x;
    msg->wrench.force.y = p_pid_y;
    msg->wrench.force.z = p_pid_z;
    msg->wrench.torque.x = p_pid_roll;
    msg->wrench.torque.z = p_pid_yaw;
    // msg->wrench.force.x = joy_->wrench.force.x;
    // msg->wrench.force.y = joy_->wrench.force.y;
    // msg->wrench.force.z = joy_->wrench.force.z;
    // msg->wrench.torque.x = joy_->wrench.torque.x;
    // msg->wrench.torque.z = joy_->wrench.torque.z;

    pub_->publish(std::move(msg));
  }
}

void TestPID::callback_target(rt_pose_plotter_msgs::msg::Targets::UniquePtr msg)
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
