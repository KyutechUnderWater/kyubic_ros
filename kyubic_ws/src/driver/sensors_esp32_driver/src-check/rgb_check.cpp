#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace sensors_esp32_driver
{

class RgbSubscriberCheck : public system_health_check::TopicSubscriberCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("sensors_esp32_driver.rgb_subscriber_check.topic_name", "/rgb");
    uint32_t timeout_ms =
      node->declare_parameter("sensors_esp32_driver.rgb_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);

    return TopicSubscriberCheckBase::check(node);
  }
};

}  // namespace sensors_esp32_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  sensors_esp32_driver::RgbSubscriberCheck, system_health_check::SystemCheckBase)
