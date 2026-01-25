#include <driver_msgs/msg/imu.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/service_server_check_base.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace localization::imu
{

using ImuMsg = driver_msgs::msg::IMU;
using OdomMsg = localization_msgs::msg::Odometry;

class ImuTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<ImuMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("localization.imu.imu_topic_status_check.topic_name", "/imu");
    uint32_t timeout_ms =
      node->declare_parameter("localization.imu.imu_topic_status_check.timeout_ms", 1000);

    set_status_id("imu_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const ImuMsg & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class ImuTransformedTopicStatusCheck
: public system_health_check::base::TopicStatusCheckBase<OdomMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.imu.imu_transformed_topic_status_check.topic_name", "/imu/odom");
    uint32_t timeout_ms = node->declare_parameter(
      "localization.imu.imu_transformed_topic_status_check.timeout_ms", 1000);

    set_status_id("imu_transformed_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const OdomMsg & msg) override
  {
    if (msg.status.imu.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class ImuResetServiceServerCheck : public system_health_check::base::ServiceServerCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.imu.imu_reset_service_server_check.service_name", "/reset");
    uint32_t timeout_ms =
      node->declare_parameter("localization.imu.imu_reset_service_server_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);
  }
};

}  // namespace localization::imu

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  localization::imu::ImuTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::imu::ImuTransformedTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::imu::ImuResetServiceServerCheck, system_health_check::base::SystemCheckBase)
