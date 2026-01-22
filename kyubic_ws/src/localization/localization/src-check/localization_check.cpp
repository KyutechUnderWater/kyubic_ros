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

class OdomTopicPubliserCheck : public system_health_check::TopicPublisherCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("localization.odom_topic_publiser_check.topic_name", "/odom");
    uint32_t timeout_ms =
      node->declare_parameter("localization.odom_topic_publiser_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1, system_health_check::ComparisonMode::EQUAL);

    return TopicPublisherCheckBase::check(node);
  }
};

class GlobalPoseTopicPubliserCheck : public system_health_check::TopicPublisherCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.global_pose_topic_publiser_check.topic_name", "/global_pose");
    uint32_t timeout_ms =
      node->declare_parameter("localization.global_pose_topic_publiser_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1, system_health_check::ComparisonMode::EQUAL);

    return TopicPublisherCheckBase::check(node);
  }
};

class GnssTopicStatusCheck : public system_health_check::TopicStatusCheckBase<GnssMsg>
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("localization.gnss_topic_status_check.topic_name", "/gnss");
    uint32_t timeout_ms =
      node->declare_parameter("localization.gnss_topic_status_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicStatusCheckBase<GnssMsg>::check(node);
  }

  bool validate(const GnssMsg & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class ImuTransformedTopicStatusCheck : public system_health_check::TopicStatusCheckBase<OdometryMsg>
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.imu_transformed_topic_status_check.topic_name", "/imu_transformed");
    uint32_t timeout_ms =
      node->declare_parameter("localization.imu_transformed_topic_status_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicStatusCheckBase<OdometryMsg>::check(node);
  }
};

class DepthOdomTopicStatusCheck : public system_health_check::TopicStatusCheckBase<OdometryMsg>
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.depth_odom_topic_status_check.topic_name", "/depth_odometry");
    uint32_t timeout_ms =
      node->declare_parameter("localization.depth_odom_topic_status_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicStatusCheckBase<OdometryMsg>::check(node);
  }
};

class DvlOdomTopicStatusCheck : public system_health_check::TopicStatusCheckBase<OdometryMsg>
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.dvl_odom_topic_status_check.topic_name", "/dvl_odometry");
    uint32_t timeout_ms =
      node->declare_parameter("localization.dvl_odom_topic_status_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicStatusCheckBase<OdometryMsg>::check(node);
  }
};

class ResetServiceServerCheck : public system_health_check::ServiceServerCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("localization.reset_service_server_check.service_name", "/reset");
    uint32_t timeout_ms =
      node->declare_parameter("localization.reset_service_server_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);

    return ServiceServerCheckBase::check(node);
  }
};

}  // namespace localization

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(localization::OdomTopicPubliserCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::GlobalPoseTopicPubliserCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(localization::GnssTopicStatusCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::ImuTransformedTopicStatusCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::DepthOdomTopicStatusCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(localization::DvlOdomTopicStatusCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(localization::ResetServiceServerCheck, system_health_check::SystemCheckBase)
