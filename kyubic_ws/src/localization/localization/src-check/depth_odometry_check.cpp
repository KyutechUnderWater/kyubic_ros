#include <driver_msgs/msg/depth.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/service_server_check_base.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace localization
{

using Msg = driver_msgs::msg::Depth;

class DepthTopicStatusCheck : public system_health_check::TopicStatusCheckBase<Msg>
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("localization.depth_topic_status_check.topic_name", "/depth");
    uint32_t timeout_ms =
      node->declare_parameter("localization.depth_topic_status_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicStatusCheckBase<Msg>::check(node);
  }

  bool validate(const Msg & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class DepthOdomTopicPubliserCheck : public system_health_check::TopicPublisherCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.depth_odom_topic_publiser_check.topic_name", "/depth_odometry");
    uint32_t timeout_ms =
      node->declare_parameter("localization.depth_odom_topic_publiser_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1, system_health_check::ComparisonMode::EQUAL);

    return TopicPublisherCheckBase::check(node);
  }
};

class DepthResetServiceServerCheck : public system_health_check::ServiceServerCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.depth_reset_service_server_check.service_name", "/reset");
    uint32_t timeout_ms =
      node->declare_parameter("localization.depth_reset_service_server_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);

    return ServiceServerCheckBase::check(node);
  }
};

}  // namespace localization

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(localization::DepthTopicStatusCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::DepthOdomTopicPubliserCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::DepthResetServiceServerCheck, system_health_check::SystemCheckBase)
