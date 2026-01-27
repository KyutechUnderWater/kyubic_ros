#include <driver_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace driver::imu_driver
{

using Msg = driver_msgs::msg::IMU;

class ImuTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<Msg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("driver.imu_driver.imu_topic_status_check.topic_name", "/imu");
    uint32_t timeout_ms =
      node->declare_parameter("driver.imu_driver.imu_topic_status_check.timeout_ms", 1000);

    set_status_id("imu_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const Msg & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

}  // namespace driver::imu_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  driver::imu_driver::ImuTopicStatusCheck, system_health_check::base::SystemCheckBase)
