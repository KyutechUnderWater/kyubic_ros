#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/topic_pub_sub_check_base.hpp>
#include <system_health_check/base_class/topic_status_check_base.hpp>

namespace planner
{

class WrenchPlanTopicPubliserCheck : public system_health_check::TopicPublisherCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "planner.wrench_plan_topic_publiser_check.topic_name", "/goal_current_odom");
    uint32_t timeout_ms =
      node->declare_parameter("planner.wrench_plan_topic_publiser_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicPublisherCheckBase::check(node);
  }
};

class VelocityPlanTopicSubscriberCheck : public system_health_check::TopicSubscriberCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name = node->declare_parameter(
      "planner.velocity_plan_topic_subscriber_check.topic_name", "/wrench_plan");
    uint32_t timeout_ms =
      node->declare_parameter("planner.velocity_plan_topic_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicSubscriberCheckBase::check(node);
  }
};

class OdomTopicSubscriberCheck : public system_health_check::TopicSubscriberCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("planner.odom_topic_subscriber_check.topic_name", "/odom");
    uint32_t timeout_ms =
      node->declare_parameter("planner.odom_topic_subscriber_check.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return TopicSubscriberCheckBase::check(node);
  }
};

}  // namespace planner

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(planner::WrenchPlanTopicPubliserCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(
  planner::VelocityPlanTopicSubscriberCheck, system_health_check::SystemCheckBase)
PLUGINLIB_EXPORT_CLASS(planner::OdomTopicSubscriberCheck, system_health_check::SystemCheckBase)
