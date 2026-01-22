#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/lifecycle_stats_check_base.hpp>

namespace joy2wrench
{

class Joy2WrenchLifecycleStatusCheck : public system_health_check::LifecycleStatusCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "joy2wrench.joy2wrench_lifecycle_status_check.lifecycle_name",
      "/joy_to_wrench_stamped_lifecycle");
    std::string expected_state = node->declare_parameter(
      "joy2wrench.joy2wrench_lifecycle_status_check.expected_state", "active");
    uint32_t timeout_ms =
      node->declare_parameter("joy2wrench.joy2wrench_lifecycle_status_check.timeout_ms", 1000);

    set_config(topic_name, expected_state, timeout_ms);

    return LifecycleStatusCheckBase::check(node);
  }
};

}  // namespace joy2wrench

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  joy2wrench::Joy2WrenchLifecycleStatusCheck, system_health_check::SystemCheckBase)
