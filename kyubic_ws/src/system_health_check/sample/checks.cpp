#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <system_health_check/base_class/action_server_check_base.hpp>
#include <system_health_check/base_class/lifecycle_stats_check_base.hpp>
#include <system_health_check/base_class/service_server_check_base.hpp>
#include <system_health_check/base_class/system_health_check_base.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace system_health_check
{

class PassCheck : public system_health_check::SystemCheckBase
{
public:
  bool check([[maybe_unused]] rclcpp::Node::SharedPtr node) override { return true; }

  std::string report([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    // Return detailed status string
    std::stringstream ss;
    ss << "Allways Pass";
    return ss.str();
  }
};

class FailCheck : public system_health_check::SystemCheckBase
{
public:
  bool check([[maybe_unused]] rclcpp::Node::SharedPtr node) override { return false; }

  std::string report([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    // Return detailed status string
    std::stringstream ss;
    ss << "Allways Fail";
    return ss.str();
  }
};

class TopicPubliserCheck : public system_health_check::TopicPublisherCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    auto topic_name = node->has_parameter("battery.topic_name")
                        ? node->get_parameter("battery.topic_name").as_string()
                        : node->declare_parameter("battery.topic_name", "/battery");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("battery.timeout_ms")
        ? node->get_parameter("battery.timeout_ms").as_int()
        : node->declare_parameter("battery.timeout_ms", 1000));

    auto count = node->has_parameter("battery.publisher_count")
                   ? node->get_parameter("battery.publisher_count").as_int()
                   : node->declare_parameter("battery.publisher_count", 1);

    auto comp_mode = node->has_parameter("battery.compare_mode")
                       ? node->get_parameter("battery.compare_mode").as_int()
                       : node->declare_parameter("battery.compare_mode", 0);

    set_config(
      topic_name, timeout_ms, count,
      comp_mode ? ComparisonMode::EQUAL : ComparisonMode::GREATER_OR_EQUAL);

    return TopicPublisherCheckBase::check(node);
  }
};

class TopicSubscriberCheck : public system_health_check::TopicSubscriberCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    auto topic_name = node->has_parameter("battery.topic_name")
                        ? node->get_parameter("battery.topic_name").as_string()
                        : node->declare_parameter("battery.topic_name", "/battery");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("battery.timeout_ms")
        ? node->get_parameter("battery.timeout_ms").as_int()
        : node->declare_parameter("battery.timeout_ms", 1000));

    auto count = node->has_parameter("battery.publisher_count")
                   ? node->get_parameter("battery.publisher_count").as_int()
                   : node->declare_parameter("battery.publisher_count", 1);

    auto comp_mode = node->has_parameter("battery.compare_mode")
                       ? node->get_parameter("battery.compare_mode").as_int()
                       : node->declare_parameter("battery.compare_mode", 0);

    set_config(
      topic_name, timeout_ms, count,
      comp_mode ? ComparisonMode::EQUAL : ComparisonMode::GREATER_OR_EQUAL);

    return TopicSubscriberCheckBase::check(node);
  }
};

class TopicStatusCheck : public system_health_check::TopicStatusCheckBase<std_msgs::msg::Bool>
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    auto topic_name = node->has_parameter("battery.topic_name")
                        ? node->get_parameter("battery.topic_name").as_string()
                        : node->declare_parameter("battery.topic_name", "/battery");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("battery.timeout_ms")
        ? node->get_parameter("battery.timeout_ms").as_int()
        : node->declare_parameter("battery.timeout_ms", 1000));

    set_config(topic_name, timeout_ms);

    return TopicStatusCheckBase<std_msgs::msg::Bool>::check(node);
  }

  bool validate(const std_msgs::msg::Bool & msg) override { return msg.data; }
};

class ServiceServerCheck : public system_health_check::ServiceServerCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    auto service_name = node->has_parameter("add_two_ints.service_name")
                          ? node->get_parameter("add_two_ints.service_name").as_string()
                          : node->declare_parameter("add_two_ints.service_name", "/add_two_ints");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("add_two_ints.timeout_ms")
        ? node->get_parameter("add_two_ints.timeout_ms").as_int()
        : node->declare_parameter("add_two_ints.timeout_ms", 1000));

    set_config(service_name, timeout_ms, 1);

    return ServiceServerCheckBase::check(node);
  }
};

class ServiceClientCheck : public system_health_check::ServiceClientCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    auto service_name = node->has_parameter("add_two_ints.service_name")
                          ? node->get_parameter("add_two_ints.service_name").as_string()
                          : node->declare_parameter("add_two_ints.service_name", "/add_two_ints");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("add_two_ints.timeout_ms")
        ? node->get_parameter("add_two_ints.timeout_ms").as_int()
        : node->declare_parameter("add_two_ints.timeout_ms", 1000));

    set_config(service_name, timeout_ms, 1);

    return ServiceClientCheckBase::check(node);
  }
};

class ActionServerCheck : public system_health_check::ActionServerCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    auto action_name = node->has_parameter("fibonacci.action_name")
                         ? node->get_parameter("fibonacci.action_name").as_string()
                         : node->declare_parameter("fibonacci.action_name", "/fibonacci");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("fibonacci.timeout_ms")
        ? node->get_parameter("fibonacci.timeout_ms").as_int()
        : node->declare_parameter("fibonacci.timeout_ms", 1000));

    set_config(action_name, timeout_ms, 1);

    return ActionServerCheckBase::check(node);
  }
};

class ActionClientCheck : public system_health_check::ActionClientCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    auto action_name = node->has_parameter("fibonacci.action_name")
                         ? node->get_parameter("fibonacci.action_name").as_string()
                         : node->declare_parameter("fibonacci.action_name", "/fibonacci");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("fibonacci.timeout_ms")
        ? node->get_parameter("fibonacci.timeout_ms").as_int()
        : node->declare_parameter("fibonacci.timeout_ms", 1000));

    set_config(action_name, timeout_ms, 1);

    return ActionClientCheckBase::check(node);
  }
};

class LifecycleStatusCheck : public system_health_check::LifecycleStatusCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string lifecycle_name = node->declare_parameter("lifecycle.lifecycle_name", "/lc_talker");
    std::string expected_state = node->declare_parameter("lifecycle.expected_state", "active");
    uint32_t timeout_ms = node->declare_parameter("lifecycle.timeout_ms", 1000);

    set_config(lifecycle_name, expected_state, timeout_ms);

    return LifecycleStatusCheckBase::check(node);
  }
};

}  // namespace system_health_check

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(system_health_check::PassCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(system_health_check::FailCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(system_health_check::TopicStatusCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::TopicPubliserCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::TopicSubscriberCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::ServiceServerCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::ServiceClientCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(system_health_check::ActionServerCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(system_health_check::ActionClientCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::LifecycleStatusCheck, system_health_check::SystemCheckBase)
