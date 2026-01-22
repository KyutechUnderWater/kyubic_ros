#include <driver_msgs/msg/dvl.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_check/base_class/topic_status_check_base.hpp>

namespace dvl_driver
{

using Msg = driver_msgs::msg::DVL;

class PowerStateTopicStatusCheck : public system_check::TopicStatusCheckBase<Msg>
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("logic_distro_rp2040_driver.power_state.topic_name", "/power_state");
    uint32_t timeout_ms =
      node->declare_parameter("logic_distro_rp2040_driver.power_state.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicStatusCheckBase<Msg>::check(node);
  }

  bool validate(const Msg & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class SystemSwitchSubscriberCheck : public system_check::TopicSubscriberCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "logic_distro_rp2040_driver.system_switch_subscriber_check.topic_name", "/system_switch");
    uint32_t timeout_ms = node->declare_parameter(
      "logic_distro_rp2040_driver.system_switch_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);

    return TopicSubscriberCheckBase::check(node);
  }
};

}  // namespace dvl_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  logic_distro_rp2040_driver::PowerStateTopicStatusCheck, system_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  logic_distro_rp2040_driver::SystemSwitchSubscriberCheck, system_check::SystemCheckBase)
