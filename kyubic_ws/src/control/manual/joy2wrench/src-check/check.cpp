#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <joy_common_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/lifecycle_stats_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace joy2wrench
{

using WrenchMsg = geometry_msgs::msg::WrenchStamped;
using JoyMsg = joy_common_msgs::msg::Joy;

class Joy2WrenchLifecycleStatusCheck : public system_health_check::base::LifecycleStatusCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "joy2wrench.joy2wrench_lifecycle_status_check.lifecycle_name",
      "/joy_to_wrench_stamped_lifecycle");
    std::string expected_state = node->declare_parameter(
      "joy2wrench.joy2wrench_lifecycle_status_check.expected_state", "active");
    uint32_t timeout_ms =
      node->declare_parameter("joy2wrench.joy2wrench_lifecycle_status_check.timeout_ms", 1000);

    set_config(topic_name, expected_state, timeout_ms);
    ;
  }
};

class RobotForceTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<WrenchMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "joy2wrench.robot_force_topic_status_check.topic_name", "/robot_force");
    uint32_t timeout_ms =
      node->declare_parameter("joy2wrench.robot_force_topic_status_check.timeout_ms", 1000);

    set_status_id("robot_force_normal");
    set_config(topic_name, timeout_ms);
  }
};

class JoyCommonTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<JoyMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("joy2wrench.joy_common_topic_status_check.topic_name", "/joy_common");
    uint32_t timeout_ms =
      node->declare_parameter("joy2wrench.joy_common_topic_status_check.timeout_ms", 1000);

    set_status_id("joy_common_normal");
    set_config(topic_name, timeout_ms);
  }
};

}  // namespace joy2wrench

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  joy2wrench::Joy2WrenchLifecycleStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  joy2wrench::RobotForceTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  joy2wrench::JoyCommonTopicStatusCheck, system_health_check::base::SystemCheckBase)
