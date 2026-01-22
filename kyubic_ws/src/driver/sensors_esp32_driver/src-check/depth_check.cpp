#include <driver_msgs/msg/depth.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace sensors_esp32_driver
{

using Msg = driver_msgs::msg::Depth;

class DepthTopicStatusCheck : public system_health_check::TopicStatusCheckBase<Msg>
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("sensors_esp32_driver.depth_topic_status_check.topic_name", "/depth");
    uint32_t timeout_ms =
      node->declare_parameter("sensors_esp32_driver.depth_topic_status_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicStatusCheckBase<Msg>::check(node);
  }

  bool validate(const Msg & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class DepthTypeSubscriberCheck : public system_health_check::TopicSubscriberCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "sensors_esp32_driver.depth_type_subscriber_check.topic_name", "/depth_type");
    uint32_t timeout_ms =
      node->declare_parameter("sensors_esp32_driver.depth_type_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);

    return TopicSubscriberCheckBase::check(node);
  }
};

}  // namespace sensors_esp32_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  sensors_esp32_driver::DepthTopicStatusCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  sensors_esp32_driver::DepthTypeSubscriberCheck, system_health_check::SystemCheckBase)
