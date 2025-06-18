/**
 * @file test_pid.cpp
 * @brief Test code for PID Controller
 * @author R.Ohnishi
 * @date 2025/05/14
 *
 * @details PID制御のテストコード
 **************************************************/

#include "test_pid/test_pid.hpp"

#include "std_msgs/msg/float32.hpp"

#include <functional>
#include <iostream>
#include <memory>

using namespace std::chrono_literals;

namespace test
{

TestPID::TestPID(const rclcpp::NodeOptions & options) : Node("test_pid", options)
{
  k = this->declare_parameter("k", 0.0);
  kp = this->declare_parameter("kp", 0.0);
  ki = this->declare_parameter("ki", 0.0);
  kd = this->declare_parameter("kd", 0.0);
  kf = this->declare_parameter("kf", 0.0);
  lo = this->declare_parameter("lo", 0.0);
  hi = this->declare_parameter("hi", 0.0);

  ppid_ = std::make_shared<pid_controller::PositionPID>(kp, ki, kd, kf);
  vpid_ = std::make_shared<pid_controller::VelocityPID>(kp, ki, kd, kf);
  p_pid_ = std::make_shared<pid_controller::P_PID>(k, kp, ki, kd, kf, lo, hi);

  odom_ = std::make_shared<localization_msgs::msg::Odometry>();

  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("robot_force", qos);
  sub_target_ = create_subscription<std_msgs::msg::Float32>(
    "target_pid", qos, std::bind(&TestPID::_callback_target, this, std::placeholders::_1));
  sub_odom_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&TestPID::_callback_odom, this, std::placeholders::_1));
  timer_ = create_wall_timer(10ms, std::bind(&TestPID::update, this));
}

void TestPID::update()
{
  if (updated) {
    updated = false;
    current = odom_->twist.linear.x;

    double ppid = ppid_->update(current, target, 0.0);
    double vpid = vpid_->update(current, target);
    // double p_pid = p_pid_->update(current, current_master target, 0.0);
    std::cout << "target: " << target << " current: " << current << " ppid: " << ppid
              << "  vpid: " << vpid << std::endl;
    std::cout << "pid: " << ppid_->get_each_term().at(0) << std::endl;

    auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
    msg->wrench.force.x = vpid;
    msg->wrench.force.y = 0.0;
    msg->wrench.force.z = 0.0;
    msg->wrench.torque.z = 0.0;
    pub_->publish(std::move(msg));
  }
}

void TestPID::_callback_target(const std_msgs::msg::Float32::UniquePtr msg) { target = msg->data; }

void TestPID::_callback_odom(const localization_msgs::msg::Odometry::UniquePtr msg)
{
  updated = true;
  odom_->twist = msg->twist;
}

}  // namespace test

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(test::TestPID)
