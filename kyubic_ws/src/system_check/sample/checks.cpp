#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <system_check/base_class/action_server_check_base.hpp>
#include <system_check/base_class/service_server_check_base.hpp>
#include <system_check/base_class/system_check_base.hpp>
#include <system_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_check/base_class/topic_status_check_base.hpp>

namespace system_check
{

class PassCheck : public system_check::SystemCheckBase
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

class FailCheck : public system_check::SystemCheckBase
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

class TopicStatusCheck : public system_check::TopicStatusCheckBase<std_msgs::msg::Bool>
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter("battery.topic_name", "/battery");
    uint32_t timeout_ms = node->declare_parameter("battery.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicStatusCheckBase<std_msgs::msg::Bool>::check(node);
  }

  bool validate(const std_msgs::msg::Bool & msg) override { return msg.data; }
};

class ServiceServerCheck : public system_check::ServiceServerCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter("add_two_ints.service_name", "/add_two_ints");
    uint32_t timeout_ms = node->declare_parameter("add_two_ints.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return ServiceServerCheckBase::check(node);
  }
};

class ActionServerCheck : public system_check::ActionServerCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter("fibonacci.action_name", "/fibonacci");
    uint32_t timeout_ms = node->declare_parameter("fibonacci.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return ActionServerCheckBase::check(node);
  }
};

}  // namespace system_check

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(system_check::PassCheck, system_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(system_check::FailCheck, system_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(system_check::TopicStatusCheck, system_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(system_check::ServiceServerCheck, system_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(system_check::ActionServerCheck, system_check::SystemCheckBase)
