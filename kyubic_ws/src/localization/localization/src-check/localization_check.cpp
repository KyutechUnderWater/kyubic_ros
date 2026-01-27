#include <driver_msgs/msg/gnss.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/service_server_check_base.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace localization
{

using GnssMsg = driver_msgs::msg::Gnss;
using OdometryMsg = localization_msgs::msg::Odometry;

class OdomTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<OdometryMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("localization.odom_topic_status_check.topic_name", "/odom");
    uint32_t timeout_ms =
      node->declare_parameter("localization.odom_topic_status_check.timeout_ms", 1000);

    set_status_id("odom_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const OdometryMsg & msg) override
  {
    if (
      msg.status.depth.id == common_msgs::msg::Status::NORMAL &&
      msg.status.dvl.id == common_msgs::msg::Status::NORMAL &&
      msg.status.imu.id == common_msgs::msg::Status::NORMAL)
      return true;
    return false;
  }
};

class GnssTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<GnssMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("localization.gnss_topic_status_check.topic_name", "/gnss");
    uint32_t timeout_ms =
      node->declare_parameter("localization.gnss_topic_status_check.timeout_ms", 1000);

    set_status_id("gnss_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const GnssMsg & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class ImuTransformedTopicStatusCheck
: public system_health_check::base::TopicStatusCheckBase<OdometryMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.imu_transformed_topic_status_check.topic_name", "/imu_transformed");
    uint32_t timeout_ms =
      node->declare_parameter("localization.imu_transformed_topic_status_check.timeout_ms", 1000);

    set_status_id("imu_transformed_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const OdometryMsg & msg) override
  {
    if (msg.status.imu.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class DepthOdomTopicStatusCheck
: public system_health_check::base::TopicStatusCheckBase<OdometryMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.depth_odom_topic_status_check.topic_name", "/depth_odometry");
    uint32_t timeout_ms =
      node->declare_parameter("localization.depth_odom_topic_status_check.timeout_ms", 1000);

    set_status_id("depth_odom_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const OdometryMsg & msg) override
  {
    if (msg.status.depth.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class DvlOdomTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<OdometryMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.dvl_odom_topic_status_check.topic_name", "/dvl_odometry");
    uint32_t timeout_ms =
      node->declare_parameter("localization.dvl_odom_topic_status_check.timeout_ms", 1000);

    set_status_id("dvl_odom_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const OdometryMsg & msg) override
  {
    if (msg.status.dvl.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class ResetServiceServerCheck : public system_health_check::base::ServiceServerCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("localization.reset_service_server_check.service_name", "/reset");
    uint32_t timeout_ms =
      node->declare_parameter("localization.reset_service_server_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);
  }
};

}  // namespace localization

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  localization::OdomTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::GnssTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::ImuTransformedTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::DepthOdomTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::DvlOdomTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::ResetServiceServerCheck, system_health_check::base::SystemCheckBase)
