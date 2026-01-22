#include <driver_msgs/msg/dvl.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/service_server_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace dvl_driver
{

using Msg = driver_msgs::msg::DVL;

class DvlTopicStatusCheck : public system_health_check::TopicStatusCheckBase<Msg>
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("dvl_driver.dvl_topic_status_check.topic_name", "/dvl");
    uint32_t timeout_ms =
      node->declare_parameter("dvl_driver.dvl_topic_status_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicStatusCheckBase<Msg>::check(node);
  }

  bool validate(const Msg & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class CommandServiceServerCheck : public system_health_check::ServiceServerCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string service_name =
      node->declare_parameter("dvl_driver.command_service_server_check.service_name", "/command");

    uint32_t timeout_ms =
      node->declare_parameter("dvl_driver.command_service_server_check.timeout_ms", 1000);

    set_config(service_name, timeout_ms, 1);

    return ServiceServerCheckBase::check(node);
  }
};

}  // namespace dvl_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dvl_driver::DvlTopicStatusCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(dvl_driver::CommandServiceServerCheck, system_health_check::SystemCheckBase)
