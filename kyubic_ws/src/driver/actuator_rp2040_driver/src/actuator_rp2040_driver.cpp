/**
 * @file actuator_rp2040_driver.cpp
 * @brief Thruster driver
 * @author K.Fujimoto
 * @date 2025/11/21
 *
 * @details スラスターの出力値を受け取り，制御マイコンに命令する
 */

#include "actuator_rp2040_driver/actuator_rp2040_driver.hpp"

#include <protolink/serial_protocol.hpp>

using namespace std::chrono_literals;

namespace actuator_rp2040_driver
{

ActuatorRP2040::ActuatorRP2040(const rclcpp::NodeOptions & options)
: Node("actuator_rp2040_driver", options),
  f_x_scale(1.0f / (4.0f * std::cos(theta_h))),
  f_y_scale(1.0f / (4.0f * std::sin(theta_h))),
  f_z_scale(1.0f / (2.0f * std::cos(theta_v))),
  t_x_scale(
    1.0f / (2.0f * std::sqrt(std::pow(dist_vy, 2) + std::pow(dist_vz, 2)) *
            std::cos(theta_h - std::atan(dist_vy / dist_vz)))),
  t_z_scale(
    1.0f / (4.0f * std::sqrt(std::pow(dist_hx, 2) + std::pow(dist_hy, 2)) *
            std::cos(theta_h - std::atan(dist_hy / dist_hx))))
{
  // Parameter declaration
  portname = this->declare_parameter("portname", "/dev/ttyUSB0");
  baudrate = this->declare_parameter("baudrate", 115200);
  max_thrust = this->declare_parameter("max_thrust", 80.0f);
  max_thrust_per = this->declare_parameter("max_thrust_per", 30.0f);
  timeout_ms = this->declare_parameter("timeout_ms", 0);

  // Initialize Heartbeat Timeout
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  // Initialize Serial Port
  try {
    port_ = protolink::serial_protocol::create_port(io_context_, portname, baudrate);
    RCLCPP_INFO(
      this->get_logger(), "ActuatorRP2040 open port: %s (%d)", portname.c_str(), baudrate);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize port: %s", e.what());
    RCLCPP_ERROR(
      this->get_logger(), "ActuatorRP2040 don't open port %s (%d)", portname.c_str(), baudrate);
    exit(1);
  }

  // Initialize Protolink Publisher
  protolink_publisher_ = std::make_shared<protolink::serial_protocol::Publisher<ProtoActuator>>(
    port_, this->get_logger());

  // Initialize ROS Publishers & Subscribers
  thruster_pub_ = this->create_publisher<driver_msgs::msg::Thruster>("thruster", 10);

  wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "robot_force", 10, std::bind(&ActuatorRP2040::wrench_callback, this, std::placeholders::_1));

  led_sub_ = this->create_subscription<driver_msgs::msg::LED>(
    "led", 10, std::bind(&ActuatorRP2040::led_callback, this, std::placeholders::_1));

  heartbeat_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "heartbeat", 10, std::bind(&ActuatorRP2040::heartbeat_callback, this, std::placeholders::_1));
}

std::array<float, ActuatorRP2040::NUM_THRUSTERS> ActuatorRP2040::_wrench2thrusts(
  float f_x, float f_y, float f_z, float t_x, float t_z)
{
  // Individual forces and torque
  float thrust_x = f_x * f_x_scale;
  float thrust_y = f_y * f_y_scale;
  float thrust_z = f_z * f_z_scale;
  float thrust_x_rot = t_x * t_x_scale;
  float thrust_z_rot = t_z * t_z_scale;

  // Calculate thrusts based on ThrusterDriver logic
  std::array<float, NUM_THRUSTERS> thrusts;
  thrusts.at(0) = -thrust_x + thrust_y + thrust_z_rot;  // HFR
  thrusts.at(1) = -thrust_x - thrust_y - thrust_z_rot;  // HFL
  thrusts.at(4) = thrust_x + thrust_y - thrust_z_rot;   // HRR
  thrusts.at(5) = thrust_x - thrust_y + thrust_z_rot;   // HRL
  thrusts.at(2) = thrust_z + thrust_x_rot;              // VL
  thrusts.at(3) = thrust_z - thrust_x_rot;              // VR
  return thrusts;
}

float ActuatorRP2040::_restrict_thrust(std::array<float, NUM_THRUSTERS> & thrusts)
{
  auto calc_total_thrust = [](const std::array<float, NUM_THRUSTERS> & t) {
    return std::accumulate(
      t.begin(), t.end(), 0.0f, [](float acc, float val) { return acc + std::abs(val); });
  };

  // If total_thrust > max_thrust, Restrict thrust
  float total_thrust = calc_total_thrust(thrusts);
  if (total_thrust > max_thrust) {
    RCLCPP_WARN(
      this->get_logger(), "total_thrust(%.2f[N]) exceeds max_thrust(%.2f[N]). Restricting.",
      total_thrust, max_thrust);

    float scale = max_thrust / total_thrust;
    for (auto & val : thrusts) val *= scale;
  }

  // Individual thruster constraints
  for (size_t i = 0; i < thrusts.size(); i++) {
    if (std::abs(thrusts[i]) > max_thrust_per) {
      RCLCPP_WARN(
        this->get_logger(), "thrusts[%lu](%.2f[N]) exceeds max_thrust_per(%.2f[N]). Restricting.",
        i, thrusts[i], max_thrust_per);
      thrusts[i] = std::clamp(thrusts[i], -max_thrust_per, max_thrust_per);
    }
  }
  return calc_total_thrust(thrusts);
}

void ActuatorRP2040::wrench_callback(geometry_msgs::msg::WrenchStamped::SharedPtr _msg)
{
  std::array<float, NUM_THRUSTERS> thrusts;
  float total_thrust = 0.0;

  // Check timeout with timer library
  if (timeout_->is_timeout(this->get_clock()->now())) {
    RCLCPP_WARN(this->get_logger(), "HeartBeat timeout. Thrusts set 0[N].");
    thrusts.fill(0.0f);
  } else {
    float force_x = _msg->wrench.force.x;
    float force_y = _msg->wrench.force.y;
    float force_z = _msg->wrench.force.z;
    float moment_x = _msg->wrench.torque.x;
    float moment_z = _msg->wrench.torque.z;

    // Calculate thrust
    thrusts = _wrench2thrusts(force_x, force_y, force_z, moment_x, moment_z);

    // Limitation (Pass by reference to modify thrusts in place)
    total_thrust = _restrict_thrust(thrusts);
  }

  // 1. Prepare and Publish ROS Message
  auto thruster_msg = std::make_unique<driver_msgs::msg::Thruster>();
  thruster_msg->thr1 = thrusts[0];
  thruster_msg->thr2 = thrusts[1];
  thruster_msg->thr3 = thrusts[2];
  thruster_msg->thr4 = thrusts[3];
  thruster_msg->thr5 = thrusts[4];
  thruster_msg->thr6 = thrusts[5];

  // 2. Prepare and Send Protolink Message
  msg_buffer_.thrusters.thr1 = thrusts[0];
  msg_buffer_.thrusters.thr2 = thrusts[1];
  msg_buffer_.thrusters.thr3 = thrusts[2];
  msg_buffer_.thrusters.thr4 = thrusts[3];
  msg_buffer_.thrusters.thr5 = thrusts[4];
  msg_buffer_.thrusters.thr6 = thrusts[5];
  ProtoActuator msg = protolink__driver_msgs__Actuator::convert(msg_buffer_);

  thruster_pub_->publish(std::move(thruster_msg));
  protolink_publisher_->send(msg);

  RCLCPP_INFO(
    this->get_logger(), "Thrust Update -> %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f SUM(%f)", thrusts[0],
    thrusts[1], thrusts[2], thrusts[3], thrusts[4], thrusts[5], total_thrust);
}

void ActuatorRP2040::led_callback(driver_msgs::msg::LED::SharedPtr _msg)
{
  msg_buffer_.leds = *_msg;
  ProtoActuator msg = protolink__driver_msgs__Actuator::convert(msg_buffer_);

  protolink_publisher_->send(msg);
}

void ActuatorRP2040::heartbeat_callback(std_msgs::msg::Bool::SharedPtr _msg)
{
  heartbeat = _msg->data;
  if (heartbeat) timeout_->reset(this->get_clock()->now());
}

}  // namespace actuator_rp2040_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(actuator_rp2040_driver::ActuatorRP2040)
