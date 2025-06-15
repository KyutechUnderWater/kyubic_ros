/**
 * @file fix_axis_plotjugger.cpp
 * @brief publish min and max value for fixed axis range of plotjugger
 * @author R.Ohnishi
 * @date 2025/06/15
 *
 * @details plotjuggerのグラフの軸範囲を指定するためのPublisher
 ********************************************************************/

#include "fix_axis_plotjugger/fix_axis_plotjugger.hpp"

using namespace std::chrono_literals;

namespace tools::plotjugger
{

FixAxisPlotjugger::FixAxisPlotjugger(const rclcpp::NodeOptions & options)
: Node("fix_axis_plotjugger", options)
{
  std::string topic_name = this->declare_parameter("topic_name", "min_max");
  min = this->declare_parameter("min", 0.0);
  max = this->declare_parameter("max", 0.0);

  rclcpp::QoS qos(rclcpp::KeepLast(1));

  pub_ = create_publisher<plotjugger_msgs::msg::MinMax>(topic_name, qos);
  timer_ = create_wall_timer(1s, std::bind(&FixAxisPlotjugger::_callback, this));
}

void FixAxisPlotjugger::_callback()
{
  auto msg = std::make_unique<plotjugger_msgs::msg::MinMax>();

  msg->min = min;
  msg->max = max;

  pub_->publish(std::move(msg));
}

}  // namespace tools::plotjugger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tools::plotjugger::FixAxisPlotjugger)
