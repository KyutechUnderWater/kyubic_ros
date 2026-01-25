#include <driver_msgs/msg/depth.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/service_server_check_base.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace localization::depth
{

using DepthMsg = driver_msgs::msg::Depth;
using OdomMsg = localization_msgs::msg::Odometry;

class DepthTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<DepthMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("localization.depth.depth_topic_status_check.topic_name", "/depth");
    uint32_t timeout_ms =
      node->declare_parameter("localization.depth.depth_topic_status_check.timeout_ms", 1000);

    set_status_id("depth_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const DepthMsg & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class DepthOdomTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<OdomMsg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.depth.depth_odom_topic_status_check.topic_name", "/depth/odom");
    uint32_t timeout_ms =
      node->declare_parameter("localization.depth.depth_odom_topic_status_check.timeout_ms", 1000);

    set_status_id("depth_odom_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const OdomMsg & msg) override
  {
    if (msg.status.depth.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class DepthResetServiceServerCheck : public system_health_check::base::ServiceServerCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.depth.depth_reset_service_server_check.service_name", "/reset");
    uint32_t timeout_ms = node->declare_parameter(
      "localization.depth.depth_reset_service_server_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);
  }
};

}  // namespace localization::depth

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  localization::depth::DepthTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::depth::DepthOdomTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::depth::DepthResetServiceServerCheck, system_health_check::base::SystemCheckBase)
