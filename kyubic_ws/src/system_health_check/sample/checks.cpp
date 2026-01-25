/**
 * @file checks.cpp
 * @brief Sample checks
 * @author R.Ohnishi
 * @date 2026/01/21
 *
 * @details サンプルチェック
 **********************************************/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <system_health_check/base_class/action_server_check_base.hpp>
#include <system_health_check/base_class/lifecycle_stats_check_base.hpp>
#include <system_health_check/base_class/service_server_check_base.hpp>
#include <system_health_check/base_class/system_check_base.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace system_health_check::checks
{

class PassCheck : public base::SystemCheckBase
{
public:
  bool check_impl([[maybe_unused]] rclcpp::Node::SharedPtr node) override { return true; }

  std::string report_impl([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    // Return detailed status string
    std::stringstream ss;
    ss << "Allways Pass";
    return ss.str();
  }
};

class FailCheck : public base::SystemCheckBase
{
public:
  bool check_impl([[maybe_unused]] rclcpp::Node::SharedPtr node) override { return false; }

  std::string report_impl([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    // Return detailed status string
    std::stringstream ss;
    ss << "Allways Fail";
    return ss.str();
  }
};

class TopicPublisherCheck : public base::TopicPublisherCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    auto topic_name = node->has_parameter("demo_nodes_cpp.topic_name")
                        ? node->get_parameter("demo_nodes_cpp.topic_name").as_string()
                        : node->declare_parameter("demo_nodes_cpp.topic_name", "/chatter");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("demo_nodes_cpp.timeout_ms")
        ? node->get_parameter("demo_nodes_cpp.timeout_ms").as_int()
        : node->declare_parameter("demo_nodes_cpp.timeout_ms", 1000));

    auto count = node->has_parameter("demo_nodes_cpp.publisher_count")
                   ? node->get_parameter("demo_nodes_cpp.publisher_count").as_int()
                   : node->declare_parameter("demo_nodes_cpp.publisher_count", 1);

    auto comp_mode = node->has_parameter("demo_nodes_cpp.compare_mode")
                       ? node->get_parameter("demo_nodes_cpp.compare_mode").as_int()
                       : node->declare_parameter("demo_nodes_cpp.compare_mode", 0);

    set_config(
      topic_name, timeout_ms, count,
      comp_mode ? base::ComparisonMode::EQUAL : base::ComparisonMode::GREATER_OR_EQUAL);
  }
};

class TopicSubscriberCheck : public base::TopicSubscriberCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    auto topic_name = node->has_parameter("demo_nodes_cpp.topic_name")
                        ? node->get_parameter("demo_nodes_cpp.topic_name").as_string()
                        : node->declare_parameter("demo_nodes_cpp.topic_name", "/chatter");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("demo_nodes_cpp.timeout_ms")
        ? node->get_parameter("demo_nodes_cpp.timeout_ms").as_int()
        : node->declare_parameter("demo_nodes_cpp.timeout_ms", 1000));

    auto count = node->has_parameter("demo_nodes_cpp.publisher_count")
                   ? node->get_parameter("demo_nodes_cpp.publisher_count").as_int()
                   : node->declare_parameter("demo_nodes_cpp.publisher_count", 1);

    auto comp_mode = node->has_parameter("demo_nodes_cpp.compare_mode")
                       ? node->get_parameter("demo_nodes_cpp.compare_mode").as_int()
                       : node->declare_parameter("demo_nodes_cpp.compare_mode", 0);

    set_config(
      topic_name, timeout_ms, count,
      comp_mode ? base::ComparisonMode::EQUAL : base::ComparisonMode::GREATER_OR_EQUAL);
  }
};

class TopicStatusCheck : public base::TopicStatusCheckBase<std_msgs::msg::String>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    auto topic_name = node->has_parameter("demo_nodes_cpp.topic_name")
                        ? node->get_parameter("demo_nodes_cpp.topic_name").as_string()
                        : node->declare_parameter("demo_nodes_cpp.topic_name", "/chatter");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("demo_nodes_cpp.timeout_ms")
        ? node->get_parameter("demo_nodes_cpp.timeout_ms").as_int()
        : node->declare_parameter("demo_nodes_cpp.timeout_ms", 1000));

    set_status_id("data_check");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const std_msgs::msg::String & msg) override { return !msg.data.empty(); }
};

class ServiceServerCheck : public base::ServiceServerCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    auto service_name = node->has_parameter("add_two_ints.service_name")
                          ? node->get_parameter("add_two_ints.service_name").as_string()
                          : node->declare_parameter("add_two_ints.service_name", "/add_two_ints");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("add_two_ints.timeout_ms")
        ? node->get_parameter("add_two_ints.timeout_ms").as_int()
        : node->declare_parameter("add_two_ints.timeout_ms", 1000));

    set_config(service_name, timeout_ms, 1);
  }
};

class ServiceClientCheck : public base::ServiceClientCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    auto service_name = node->has_parameter("add_two_ints.service_name")
                          ? node->get_parameter("add_two_ints.service_name").as_string()
                          : node->declare_parameter("add_two_ints.service_name", "/add_two_ints");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("add_two_ints.timeout_ms")
        ? node->get_parameter("add_two_ints.timeout_ms").as_int()
        : node->declare_parameter("add_two_ints.timeout_ms", 1000));

    set_config(service_name, timeout_ms, 1);
  }
};

class ActionServerCheck : public base::ActionServerCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    auto action_name = node->has_parameter("fibonacci.action_name")
                         ? node->get_parameter("fibonacci.action_name").as_string()
                         : node->declare_parameter("fibonacci.action_name", "/fibonacci");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("fibonacci.timeout_ms")
        ? node->get_parameter("fibonacci.timeout_ms").as_int()
        : node->declare_parameter("fibonacci.timeout_ms", 1000));

    set_config(action_name, timeout_ms, 1);
  }
};

class ActionClientCheck : public base::ActionClientCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    auto action_name = node->has_parameter("fibonacci.action_name")
                         ? node->get_parameter("fibonacci.action_name").as_string()
                         : node->declare_parameter("fibonacci.action_name", "/fibonacci");

    auto timeout_ms = static_cast<uint32_t>(
      node->has_parameter("fibonacci.timeout_ms")
        ? node->get_parameter("fibonacci.timeout_ms").as_int()
        : node->declare_parameter("fibonacci.timeout_ms", 1000));

    set_config(action_name, timeout_ms, 1);
  }
};

class LifecycleStatusCheck : public base::LifecycleStatusCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string lifecycle_name = node->declare_parameter("lifecycle.lifecycle_name", "/lc_talker");
    std::string expected_state = node->declare_parameter("lifecycle.expected_state", "active");
    uint32_t timeout_ms = node->declare_parameter("lifecycle.timeout_ms", 1000);

    set_config(lifecycle_name, expected_state, timeout_ms);
  }
};

}  // namespace system_health_check::checks

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  system_health_check::checks::PassCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::checks::FailCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::checks::TopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::checks::TopicPublisherCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::checks::TopicSubscriberCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::checks::ServiceServerCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::checks::ServiceClientCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::checks::ActionServerCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::checks::ActionClientCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  system_health_check::checks::LifecycleStatusCheck, system_health_check::base::SystemCheckBase)
