#include <driver_msgs/msg/system_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace sensors_esp32_driver
{

using Msg = driver_msgs::msg::SystemStatus;

class SystemStatusTopicStatusCheck : public system_health_check::TopicStatusCheckBase<Msg>
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "sensors_esp32_driver.system_status_topic_status_check.topic_name", "/system_status");
    uint32_t timeout_ms = node->declare_parameter(
      "sensors_esp32_driver.system_status_topic_status_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicStatusCheckBase<Msg>::check(node);
  }

  bool validate(const Msg & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

}  // namespace sensors_esp32_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  sensors_esp32_driver::SystemStatusTopicStatusCheck, system_health_check::SystemCheckBase)
