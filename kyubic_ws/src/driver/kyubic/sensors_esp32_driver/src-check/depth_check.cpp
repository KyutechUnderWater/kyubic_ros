#include <driver_msgs/msg/depth.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace driver::sensors_esp32_driver
{

using Msg = driver_msgs::msg::Depth;

class DepthTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<Msg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "driver.sensors_esp32_driver.depth_topic_status_check.topic_name", "/depth");
    uint32_t timeout_ms = node->declare_parameter(
      "driver.sensors_esp32_driver.depth_topic_status_check.timeout_ms", 1000);

    set_status_id("depth_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const Msg & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class DepthTypeSubscriberCheck : public system_health_check::base::TopicSubscriberCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "driver.sensors_esp32_driver.depth_type_subscriber_check.topic_name", "/depth_type");
    uint32_t timeout_ms = node->declare_parameter(
      "driver.sensors_esp32_driver.depth_type_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);
  }
};

}  // namespace driver::sensors_esp32_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  driver::sensors_esp32_driver::DepthTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  driver::sensors_esp32_driver::DepthTypeSubscriberCheck,
  system_health_check::base::SystemCheckBase)
