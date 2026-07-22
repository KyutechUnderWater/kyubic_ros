#include <driver_msgs/msg/power_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace driver::logic_distro_rp2040_driver
{

using Msg = driver_msgs::msg::PowerState;

class PowerStateTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<Msg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "driver.logic_distro_rp2040_driver.power_state_topic_status_check.topic_name",
      "/power_state");
    uint32_t timeout_ms = node->declare_parameter(
      "driver.logic_distro_rp2040_driver.power_state_topic_status_check.timeout_ms", 1000);

    set_status_id("power_state_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const Msg & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class SystemSwitchSubscriberCheck : public system_health_check::base::TopicSubscriberCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "driver.logic_distro_rp2040_driver.system_switch_subscriber_check.topic_name",
      "/system_switch");
    uint32_t timeout_ms = node->declare_parameter(
      "driver.logic_distro_rp2040_driver.system_switch_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);
  }
};

}  // namespace driver::logic_distro_rp2040_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  driver::logic_distro_rp2040_driver::PowerStateTopicStatusCheck,
  system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  driver::logic_distro_rp2040_driver::SystemSwitchSubscriberCheck,
  system_health_check::base::SystemCheckBase)
