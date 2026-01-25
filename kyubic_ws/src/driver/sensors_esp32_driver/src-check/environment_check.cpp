#include <driver_msgs/msg/environment.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace driver::sensors_esp32_driver
{

using Msg = driver_msgs::msg::Environment;

class EnvironmentTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<Msg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "driver.sensors_esp32_driver.environment_topic_status_check.topic_name", "/environment");
    uint32_t timeout_ms = node->declare_parameter(
      "driver.sensors_esp32_driver.environment_topic_status_check.timeout_ms", 1000);

    set_status_id("environment_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const Msg & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

}  // namespace driver::sensors_esp32_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  driver::sensors_esp32_driver::EnvironmentTopicStatusCheck,
  system_health_check::base::SystemCheckBase)
