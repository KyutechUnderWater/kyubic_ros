#include <driver_msgs/msg/power_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_check/base_class/topic_status_check_base.hpp>

namespace actuator_rp2040_driver
{

using Msg = driver_msgs::msg::PowerState;

class ThrusterPublisherCheck : public system_check::TopicPublisherCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "actuator_rp2040_driver.thruster_publisher_check.topic_name", "/system_switch");
    uint32_t timeout_ms =
      node->declare_parameter("actuator_rp2040_driver.thruster_publisher_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);

    return TopicPublisherCheckBase::check(node);
  }
};

class WrenchSubscriberCheck : public system_check::TopicSubscriberCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "actuator_rp2040_driver.wrench_subscriber_check.topic_name", "/system_switch");
    uint32_t timeout_ms =
      node->declare_parameter("actuator_rp2040_driver.wrench_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);

    return TopicSubscriberCheckBase::check(node);
  }
};

class LedSubscriberCheck : public system_check::TopicSubscriberCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "actuator_rp2040_driver.led_subscriber_check.topic_name", "/system_switch");
    uint32_t timeout_ms =
      node->declare_parameter("actuator_rp2040_driver.led_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);

    return TopicSubscriberCheckBase::check(node);
  }
};

class HaertbeatSubscriberCheck : public system_check::TopicSubscriberCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "actuator_rp2040_driver.heartbeat_subscriber_check.topic_name", "/system_switch");
    uint32_t timeout_ms =
      node->declare_parameter("actuator_rp2040_driver.heartbeat_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);

    return TopicSubscriberCheckBase::check(node);
  }
};

}  // namespace actuator_rp2040_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  actuator_rp2040_driver::ThrusterPublisherCheck, system_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(actuator_rp2040_driver::WrenchSubscriberCheck, system_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(actuator_rp2040_driver::LedSubscriberCheck, system_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  actuator_rp2040_driver::HaertbeatSubscriberCheck, system_check::SystemCheckBase)
