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
  mlo = this->declare_parameter("mlo", 0.0);
  mhi = this->declare_parameter("mhi", 0.0);
  slo = this->declare_parameter("slo", 0.0);
  shi = this->declare_parameter("shi", 0.0);

  ppid_ = std::make_shared<pid_controller::PositionPID>(kp, ki, kd, kf);
  vpid_ = std::make_shared<pid_controller::VelocityPID>(kp, ki, kd, kf, slo, shi);
  p_pid_ = std::make_shared<pid_controller::VelocityP_PID>(k, kp, ki, kd, kf, mlo, mhi, slo, shi);

  odom_ = std::make_shared<localization_msgs::msg::Odometry>();

  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("robot_force", qos);
  sub_target_ = create_subscription<std_msgs::msg::Float32>(
    "target_pid", qos, std::bind(&TestPID::callback_target, this, std::placeholders::_1));
  sub_odom_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&TestPID::callback_odom, this, std::placeholders::_1));
  timer_ = create_wall_timer(10ms, std::bind(&TestPID::update, this));
}

void TestPID::update()
{
  if (updated) {
    updated = false;

    double vpid_x, vpid_y, vpid_z, vpid_yaw;
    vpid_x = vpid_y = vpid_z = vpid_yaw = 0.0;
    // // x-axis
    // vpid_x = vpid_->update(odom_->twist.linear.x, target);
    //
    // // y-axis
    // vpid_y = vpid_->update(odom_->twist.linear.y, target);
    //
    // z-axis
    // vpid_z = vpid_->update(odom_->twist.linear.z_depth, target);
    // vpid_z = -vpid_->update(odom_->twist.linear.z_altitude, target);
    // if (odom_->pose.position.z_altitude < 2) {
    //   vpid_z = 0.0;
    // }

    // yaw-axis
    // vpid_yaw = vpid_->update(odom_->twist.angular.z, target);

    // double ppid = ppid_->update(, target, 0.0);
    // double p_pid_yaw = p_pid_->update(odom_->twist.angular.z, odom_->pose.orientation.z, target);

    if (target - odom_->pose.orientation.z < 180) target += 360;
    if (target - odom_->pose.orientation.z > 180) target -= 360;
    double p_pid_yaw = p_pid_->update(odom_->twist.angular.z, odom_->pose.orientation.z, target);

    std::cout << "target: " << target << " current: " << odom_->twist.angular.z
              << " current_t: " << odom_->pose.orientation.z << " p_pid: " << p_pid_yaw
              << std::endl;

    // Create message from pid
    auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
    msg->wrench.force.x = vpid_x;
    msg->wrench.force.y = vpid_y;
    msg->wrench.force.z = vpid_z;
    msg->wrench.torque.z = p_pid_yaw;

    pub_->publish(std::move(msg));
  }
}

void TestPID::callback_target(const std_msgs::msg::Float32::UniquePtr msg) { target = msg->data; }

void TestPID::callback_odom(localization_msgs::msg::Odometry::UniquePtr msg)
{
  updated = true;
  odom_ = std::move(msg);
}

}  // namespace test

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(test::TestPID)
