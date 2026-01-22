#include <joy_common_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace joy_common
{

using Msg = joy_common_msgs::msg::Joy;

class JoyCommonTopicStatusCheck : public system_health_check::TopicStatusCheckBase<Msg>
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("joy_common.joy_common_topic_status_check.topic_name", "/joy_common");
    uint32_t timeout_ms =
      node->declare_parameter("joy_common.joy_common_topic_status_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicStatusCheckBase<Msg>::check(node);
  }
};

class JoyTopicSubscriberCheck : public system_health_check::TopicSubscriberCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("joy_common.joy_topic_subscriber.topic_name", "/joy");
    uint32_t timeout_ms =
      node->declare_parameter("joy_common.joy_topic_subscriber.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicSubscriberCheckBase::check(node);
  }
};

}  // namespace joy_common

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(joy_common::JoyCommonTopicStatusCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(joy_common::JoyTopicSubscriberCheck, system_health_check::SystemCheckBase)
