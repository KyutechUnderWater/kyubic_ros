#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>

namespace driver::sensors_esp32_driver
{

class BuzzerSwitchTopicSubscriberCheck : public system_health_check::base::TopicSubscriberCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "driver.sensors_esp32_driver.buzzer_switch_topic_subscriber_check.topic_name",
      "/buzzer_switch");
    uint32_t timeout_ms = node->declare_parameter(
      "driver.sensors_esp32_driver.buzzer_switch_topic_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);
    ;
  }
};

}  // namespace driver::sensors_esp32_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  driver::sensors_esp32_driver::BuzzerSwitchTopicSubscriberCheck,
  system_health_check::base::SystemCheckBase)
