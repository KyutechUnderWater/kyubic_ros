#include <joy_common_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace joy_common
{

using JoyCommonMsg = joy_common_msgs::msg::Joy;
using JoyMsg = sensor_msgs::msg::Joy;

class JoyCommonTopicStatusCheck
: public system_health_check::base::TopicStatusCheckBase<JoyCommonMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("joy_common.joy_common_topic_status_check.topic_name", "/joy_common");
    uint32_t timeout_ms =
      node->declare_parameter("joy_common.joy_common_topic_status_check.timeout_ms", 1000);

    set_status_id("joy_common_normal");
    set_config(topic_name, timeout_ms);
  }
};

class JoyTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<JoyMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("joy_common.joy_topic_status_check.topic_name", "/joy");
    uint32_t timeout_ms =
      node->declare_parameter("joy_common.joy_topic_status_check.timeout_ms", 1000);

    set_status_id("joy_common_normal");
    set_config(topic_name, timeout_ms);
  }
};

}  // namespace joy_common

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  joy_common::JoyCommonTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(joy_common::JoyTopicStatusCheck, system_health_check::base::SystemCheckBase)
