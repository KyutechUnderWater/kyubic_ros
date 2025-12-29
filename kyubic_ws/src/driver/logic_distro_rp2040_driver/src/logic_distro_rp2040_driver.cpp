/**
* @file logic_distro_rp2040_driver.hpp
* @brief  Get voltage, current, and power. Set system state
* @author R.Ohnishi
* @date 2025/11/21
*
* @details それぞれの電源の状態を取得し，トピックに流す
******************************************************/

#include "logic_distro_rp2040_driver/logic_distro_rp2040_driver.hpp"

using namespace std::chrono_literals;

namespace logic_distro_rp2040_driver
{

LogicDistroRP2040::LogicDistroRP2040(const rclcpp::NodeOptions & options)
: Node("logic_distro_rp2040_driver", options)
{
  portname = this->declare_parameter("portname", "/dev/ttyUSB0");
  baudrate = this->declare_parameter("baudrate", 115200);
  mcu_ip_addr = this->declare_parameter("mcu_ip_addr", "192.168.9.5");
  mcu_port = this->declare_parameter("mcu_port", 9000);
  this_port = this->declare_parameter("this_port", 9000);
  timeout_ms = this->declare_parameter("timeout", 1000);

  try {
    port_ = protolink::serial_protocol::create_port(io_context_, portname, baudrate);
    RCLCPP_INFO(
      this->get_logger(), "LogicDistroRP2040 open port: %s (%d)", portname.c_str(), baudrate);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize udp publisher: %s", e.what());
    RCLCPP_ERROR(
      this->get_logger(), "LogicDistroRP2040 don't open port %s (%d)", portname.c_str(), baudrate);
    exit(1);
  }
  try {
    sock_ = protolink::udp_protocol::create_socket(io_context_, this_port);
    RCLCPP_INFO(this->get_logger(), "LogicDistroRP2040 open port: %d", this_port);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize udp publisher: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "LogicDistroRP2040 don't open port %d", this_port);
    exit(1);
  }
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms * 1e6);

  rclcpp::QoS qos(rclcpp::KeepLast(10));

  pub_ = create_publisher<driver_msgs::msg::PowerState>("power_state", rclcpp::SensorDataQoS());

  sub_ = create_subscription<driver_msgs::msg::SystemSwitch>(
    "system_switch", qos, std::bind(&LogicDistroRP2040::ros_callback, this, std::placeholders::_1));

  protolink_publisher_udp_ = std::make_shared<protolink::udp_protocol::Publisher<ProtoPowerState>>(
    sock_, mcu_ip_addr, mcu_port, this->get_logger());

  protolink_publisher_ = std::make_shared<protolink::serial_protocol::Publisher<ProtoSystemSwitch>>(
    port_, this->get_logger());

  protolink_subscriber_ = std::make_shared<protolink::serial_protocol::Subscriber<ProtoPowerState>>(
    port_, std::bind(&LogicDistroRP2040::protolink_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(100ms, std::bind(&LogicDistroRP2040::_check_timeout, this));
}

void LogicDistroRP2040::ros_callback(const driver_msgs::msg::SystemSwitch & _msg)
{
  ProtoSystemSwitch proto_msg = protolink__driver_msgs__SystemSwitch::convert(_msg);

  if (protolink_publisher_) {
    protolink_publisher_->send(proto_msg);

    RCLCPP_DEBUG(this->get_logger(), "Sent SystemSwitch command via serial");
  } else {
    RCLCPP_WARN(this->get_logger(), "Protolink Publisher is not initialized");
  }
}

void LogicDistroRP2040::protolink_callback(const ProtoPowerState & _msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timeout_->reset(this->get_clock()->now());

  driver_msgs::msg::PowerState msg = protolink__driver_msgs__PowerState::convert(_msg);
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "power_link";

  protolink_publisher_udp_->send(_msg);
  pub_->publish(msg);
  RCLCPP_INFO(
    this->get_logger(), "Publish ---> Logic vol: %f  Act vol: %f", _msg.log_voltage(),
    _msg.act_voltage());
}

void LogicDistroRP2040::_check_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (timeout_->is_timeout(this->get_clock()->now())) {
    auto msg = std::make_unique<driver_msgs::msg::PowerState>();
    msg->header.stamp = this->get_clock()->now();
    msg->status.id = common_msgs::msg::Status::ERROR;

    pub_->publish(std::move(msg));

    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), timeout_->get_timeout() * 1e-6,
      "PowerState driver timeout: %lu [ns]", timeout_->get_elapsed_time());
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), timeout_->get_timeout() * 1e-6,
      "Failed to get PowerState data");
    return;
  }
}

}  // namespace logic_distro_rp2040_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(logic_distro_rp2040_driver::LogicDistroRP2040)
