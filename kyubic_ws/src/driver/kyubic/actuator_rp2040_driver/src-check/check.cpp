#include <driver_msgs/msg/power_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace driver::actuator_rp2040_driver
{

class ThrusterPublisherCheck : public system_health_check::base::TopicPublisherCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "driver.actuator_rp2040_driver.thruster_publisher_check.topic_name", "/thruster");
    uint32_t timeout_ms = node->declare_parameter(
      "driver.actuator_rp2040_driver.thruster_publisher_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1, system_health_check::base::ComparisonMode::EQUAL);
  }
};

class RobotForceSubscriberCheck : public system_health_check::base::TopicSubscriberCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "driver.actuator_rp2040_driver.robot_force_subscriber_check.topic_name", "/robot_force");
    uint32_t timeout_ms = node->declare_parameter(
      "driver.actuator_rp2040_driver.robot_force_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1, system_health_check::base::ComparisonMode::EQUAL);
  }
};

class LedSubscriberCheck : public system_health_check::base::TopicSubscriberCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "driver.actuator_rp2040_driver.led_subscriber_check.topic_name", "/led");
    uint32_t timeout_ms = node->declare_parameter(
      "driver.actuator_rp2040_driver.led_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1, system_health_check::base::ComparisonMode::EQUAL);
  }
};

class HeartbeatSubscriberCheck : public system_health_check::base::TopicSubscriberCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "driver.actuator_rp2040_driver.heartbeat_subscriber_check.topic_name", "/heartbeat");
    uint32_t timeout_ms = node->declare_parameter(
      "driver.actuator_rp2040_driver.heartbeat_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1, system_health_check::base::ComparisonMode::EQUAL);
  }
};

}  // namespace driver::actuator_rp2040_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  driver::actuator_rp2040_driver::ThrusterPublisherCheck,
  system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  driver::actuator_rp2040_driver::RobotForceSubscriberCheck,
  system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  driver::actuator_rp2040_driver::LedSubscriberCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  driver::actuator_rp2040_driver::HeartbeatSubscriberCheck,
  system_health_check::base::SystemCheckBase)
