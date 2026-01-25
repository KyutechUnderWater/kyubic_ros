#include <driver_msgs/msg/button_battery_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace driver::sensors_esp32_driver
{

using Msg = driver_msgs::msg::ButtonBatteryState;

class ButtonBatteryStateTopicStatusCheck
: public system_health_check::base::TopicStatusCheckBase<Msg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "driver.sensors_esp32_driver.button_battery_state_topic_status_check.topic_name",
      "/power_state");
    uint32_t timeout_ms = node->declare_parameter(
      "driver.sensors_esp32_driver.button_battery_state_topic_status_check.timeout_ms", 1000);

    set_status_id("button_battery_state_status");
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
  driver::sensors_esp32_driver::ButtonBatteryStateTopicStatusCheck,
  system_health_check::base::SystemCheckBase)
