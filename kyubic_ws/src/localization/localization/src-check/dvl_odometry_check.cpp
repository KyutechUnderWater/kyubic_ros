#include <driver_msgs/msg/dvl.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/service_server_check_base.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace localization::dvl
{

using DvlMsg = driver_msgs::msg::DVL;
using OdomMsg = localization_msgs::msg::Odometry;

class DvlTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<DvlMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("localization.dvl.dvl_topic_status_check.topic_name", "/dvl");
    uint32_t timeout_ms =
      node->declare_parameter("localization.dvl.dvl_topic_status_check.timeout_ms", 1000);

    set_status_id("dvl_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const DvlMsg & msg) override
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
      "localization.dvl.imu_transformed_topic_status_check.topic_name", "/imu/transformed");
    uint32_t timeout_ms = node->declare_parameter(
      "localization.dvl.imu_transformed_topic_status_check.timeout_ms", 1000);

    set_status_id("imu_transformed_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const OdomMsg & msg) override
  {
    if (msg.status.imu.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class DvlOdomTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<OdomMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.dvl.dvl_odom_topic_status_check.topic_name", "/dvl/odom");
    uint32_t timeout_ms =
      node->declare_parameter("localization.dvl.dvl_odom_topic_status_check.timeout_ms", 1000);

    set_status_id("dvl_odom_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const OdomMsg & msg) override
  {
    if (msg.status.dvl.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class DvlResetServiceServerCheck : public system_health_check::base::ServiceServerCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.dvl.dvl_reset_service_server_check.service_name", "/reset");
    uint32_t timeout_ms =
      node->declare_parameter("localization.dvl.dvl_reset_service_server_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);
  }
};

}  // namespace localization::dvl

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  localization::dvl::DvlTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::dvl::ImuTransformedTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::dvl::DvlOdomTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::dvl::DvlResetServiceServerCheck, system_health_check::base::SystemCheckBase)
