#include <localization_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace planner::wrench_planner
{

using Msg = localization_msgs::msg::Odometry;

class WrenchPlanTopicPublisherCheck : public system_health_check::base::TopicPublisherCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "planner.wrench_planner.wrench_plan_topic_publisher_check.topic_name", "/goal_current_odom");
    uint32_t timeout_ms = node->declare_parameter(
      "planner.wrench_planner.wrench_plan_topic_publisher_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);
  }
};

class ZohWrenchPlanTopicSubscriberCheck : public system_health_check::base::TopicSubscriberCheckBase
{
private:
  void prepare_check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "planner.wrench_planner.zoh_wrench_plan_topic_subscriber_check.topic_name",
      "/zoh_wrench_plan");
    uint32_t timeout_ms = node->declare_parameter(
      "planner.wrench_planner.zoh_wrench_plan_topic_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);
    ;
  }
};

}  // namespace planner::wrench_planner

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  planner::wrench_planner::WrenchPlanTopicPublisherCheck,
  system_health_check::base::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  planner::wrench_planner::ZohWrenchPlanTopicSubscriberCheck,
  system_health_check::base::SystemCheckBase)
