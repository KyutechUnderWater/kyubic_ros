#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/action_server_check_base.hpp>

namespace planner
{

class PDLAPlannerActionServerCheck : public system_health_check::ActionServerCheckBase
{
public:
  bool check(rclcpp::Node::SharedPtr node) override
  {
    std::string topic_name =
      node->declare_parameter("planner.pdla_planner.action_name", "/pdla_planner");
    uint32_t timeout_ms = node->declare_parameter("planner.pdla_planner.timeout_ms", 1000);

    set_config(topic_name, timeout_ms);

    return ActionServerCheckBase::check(node);
  }
};

}  // namespace planner

// PLUGINLIB_EXPORT_CLASS(class name, base class name)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(planner::PDLAPlannerActionServerCheck, system_health_check::SystemCheckBase)
