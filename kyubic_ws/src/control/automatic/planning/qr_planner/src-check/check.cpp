#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/action_server_check_base.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace planner::qr_planner
{

using Msg = localization_msgs::msg::Odometry;

class OdomTopicStatusCheck : public system_health_check::base::TopicStatusCheckBase<Msg>
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("planner.qr_planner.odom_topic_status_check.topic_name", "/odom");
    uint32_t timeout_ms =
      node->declare_parameter("planner.qr_planner.odom_topic_status_check.timeout_ms", 1000);

    set_status_id("odom_status");
    set_config(topic_name, timeout_ms);
  }

  bool validate(const Msg & msg) override
  {
    if (
      msg.status.depth.id == common_msgs::msg::Status::NORMAL &&
      msg.status.dvl.id == common_msgs::msg::Status::NORMAL &&
      msg.status.imu.id == common_msgs::msg::Status::NORMAL)
      return true;
    return false;
  }
};

class QRPlannerActionServerCheck : public system_health_check::base::ActionServerCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "planner.qr_planner.qr_planner_action_server_check.action_name", "/qr_planner");
    uint32_t timeout_ms =
      node->declare_parameter("planner.qr_planner.qr_planner_action_server_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms, 1);
  }
};

}  // namespace planner::qr_planner

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  planner::qr_planner::OdomTopicStatusCheck, system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  planner::qr_planner::QRPlannerActionServerCheck, system_health_check::base::SystemCheckBase)
