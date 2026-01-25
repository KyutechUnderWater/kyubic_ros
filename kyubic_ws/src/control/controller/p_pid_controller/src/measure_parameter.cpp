/**
 * @file measure_parameter.cpp
 * @brief measure offset for P_PID Controller
 * @author R.Ohnishi
 * @date 2025/11/05
 *
 * @details rollのモーメントとz-axisの浮力を計測する
 **************************************************/

#include "p_pid_controller/measure_parameter.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <numeric>

using namespace std::chrono_literals;

namespace controller
{

MeasureParam::MeasureParam(const rclcpp::NodeOptions & options) : Node("measure_parameter", options)
{
  yaml_path = this->declare_parameter("pid_gain_yaml", "");

  try {
    p_pid_ctrl_ = std::make_shared<controller::P_PIDController>(yaml_path);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "%s", e.what());
    rclcpp::shutdown();
  }

  p_pid_ctrl_->set_z_offset(0);
  p_pid_ctrl_->set_roll_offset(0);

  // Create messages instance
  odom_ = std::make_shared<localization_msgs::msg::Odometry>();

  // ROS 2 communication
  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("robot_force", qos);
  sub_odom_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&MeasureParam::callback_odom, this, std::placeholders::_1));
}

bool MeasureParam::_measure_z()
{
  size_t idx = count++ % buf_size;
  buf_sens[idx] = odom_->pose.position.z_depth;
  buf_out[idx] = force_z;

  // Mean Absolute Deviation
  double mad_depth =
    std::accumulate(
      buf_sens.begin(), buf_sens.end(), 0.0,
      [this](double acc, double val) { return acc + std::abs(val - target_z_depth); }) /
    static_cast<double>(buf_sens.size());

  // If depth is stable, calculate force avelage
  if (mad_depth < 0.05) {
    param_z =
      std::accumulate(buf_out.begin(), buf_out.end(), 0.0) / static_cast<double>(buf_out.size());
    return true;
  }
  return false;
}

bool MeasureParam::_measure_roll()
{
  size_t idx = count++ % buf_size;
  buf_sens[idx] = odom_->pose.orientation.x;
  buf_out[idx] = torque_roll / sin(odom_->pose.orientation.x * std::numbers::pi / 180.0);

  // Mean Absolute Deviation
  double mad_roll =
    std::accumulate(
      buf_sens.begin(), buf_sens.end(), 0.0,
      [this](double acc, double val) { return acc + std::abs(val - target_roll); }) /
    static_cast<double>(buf_sens.size());

  // If roll is stable, calculate torque avelage
  if (mad_roll < 5) {
    param_roll =
      std::accumulate(buf_out.begin(), buf_out.end(), 0.0) / static_cast<double>(buf_out.size());
    return true;
  }
  return false;
}

void MeasureParam::_save_yaml()
{
  try {
    YAML::Node config = YAML::LoadFile(yaml_path);

    config["z"]["vpid"]["offset"] = param_z;
    config["roll"]["vpid"]["roll_scale_factor"] = param_roll;
    RCLCPP_INFO(this->get_logger(), "Write z.vpid.offset = %f", param_z);
    RCLCPP_INFO(this->get_logger(), "Write roll.vpid.roll_scale_factor = %f", param_roll);

    // Overrite
    std::ofstream fout(yaml_path);
    fout << config;
    fout.close();

  } catch (const YAML::Exception & e) {
    RCLCPP_FATAL(this->get_logger(), "YAML Error: %s", e.what());
  }
}

void MeasureParam::measure()
{
  if (z_measuring == false && roll_measuring == false) {
    z_measuring = true;
    target_z_depth = 0.2;
    start_time = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Z-Axis measurement: ...");
  }

  if (z_measuring) {
    if (z_measured = _measure_z(); z_measured) {
      RCLCPP_INFO(get_logger(), "Z-Axis measurement: successfull. param_z is  %f[N]", param_z);
      z_measuring = false;
    } else if ((this->get_clock()->now() - start_time).seconds() > timeout) {
      RCLCPP_ERROR(this->get_logger(), "Z-Axis measurement: faild");
      rclcpp::shutdown();
    }
  }

  if (z_measuring == false && roll_measuring == false) {
    roll_measuring = true;
    target_roll = 15;
    start_time = this->get_clock()->now();
    buf_sens = {0};
    buf_out = {0};
    RCLCPP_INFO(this->get_logger(), "Roll measurement: ...");
  }

  if (roll_measuring) {
    if (roll_measured = _measure_roll(); roll_measured) {
      RCLCPP_INFO(get_logger(), "Roll measurement: successfull. param_roll is %f[Nm]", param_roll);
      _save_yaml();
      rclcpp::shutdown();
    } else if ((this->get_clock()->now() - start_time).seconds() > timeout) {
      RCLCPP_ERROR(this->get_logger(), "Roll measurement: faild");
      rclcpp::shutdown();
    }
  }
}

void MeasureParam::pid_update()
{
  const auto & odom_status = odom_->status;
  const auto & current_pose = odom_->pose;
  const auto & current_twst = odom_->twist;

  auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();

  if (
    odom_status.depth.id == common_msgs::msg::Status::ERROR ||
    odom_status.imu.id == common_msgs::msg::Status::ERROR) {
    RCLCPP_ERROR(this->get_logger(), "The current odometry is invalid");

    double force_z = 0.0;
    {
      msg->wrench.force.z = force_z;
      RCLCPP_WARN(this->get_logger(), "only z-axis control: %lf[N]", force_z);
    }
  } else {
    double force_z = p_pid_ctrl_->pid_z_update(
      current_twst.linear.z_depth, current_pose.position.z_depth, target_z_depth);

    double torque_x =
      p_pid_ctrl_->pid_roll_update(current_twst.angular.x, current_pose.orientation.x, target_roll);

    RCLCPP_DEBUG(
      this->get_logger(), "P-PID -> x: %f  y: %f  z: %f  z_mode: %u  roll: %f  yaw: %f", 0.0, 0.0,
      force_z, 0, torque_x, 0.0);

    {
      this->force_z = force_z;
      this->torque_roll = torque_x;
      msg->wrench.force.z = force_z;
      msg->wrench.torque.x = torque_x;
    }
  }
  msg->header.stamp = this->get_clock()->now();
  pub_->publish(std::move(msg));
}

void MeasureParam::callback_odom(localization_msgs::msg::Odometry::UniquePtr msg)
{
  odom_ = std::move(msg);
  measure();
  pid_update();
}

}  // namespace controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(controller::MeasureParam)
