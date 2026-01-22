#include <driver_msgs/msg/dvl.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/service_server_check_base.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace localization
{

using MsgDVL = driver_msgs::msg::DVL;

class DvlTopicStatusCheck : public system_health_check::TopicStatusCheckBase<MsgDVL>
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("localization.dvl_topic_status_check.topic_name", "/dvl");
    uint32_t timeout_ms =
      node->declare_parameter("localization.dvl_topic_status_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicStatusCheckBase<MsgDVL>::check(node);
  }

  bool validate(const MsgDVL & msg) override
  {
    if (msg.status.id == common_msgs::msg::Status::NORMAL) return true;
    return false;
  }
};

class DvlOdomTopicPubliserCheck : public system_health_check::TopicPublisherCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "localization.dvl_odom_topic_publiser_check.topic_name", "/depth_odometry");
    uint32_t timeout_ms =
      node->declare_parameter("localization.dvl_odom_topic_publiser_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1, system_health_check::ComparisonMode::EQUAL);

    return TopicPublisherCheckBase::check(node);
  }
};

class DvlResetServiceServerCheck : public system_health_check::ServiceServerCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("localization.dvl_reset_service_server_check.service_name", "/reset");
    uint32_t timeout_ms =
      node->declare_parameter("localization.dvl_reset_service_server_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);

    return ServiceServerCheckBase::check(node);
  }
};

}  // namespace localization

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  localization::DvlOdomTopicPubliserCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  localization::DvlResetServiceServerCheck, system_health_check::SystemCheckBase)
