/**
 * @file fix_axis_plotjugger.hpp
 * @brief publish min and max value for fixed axis range of plotjugger
 * @author R.Ohnishi
 * @date 2025/06/15
 *
 * @details plotjuggerのグラフの軸範囲を指定するためのPublisher
 ********************************************************************/

#ifndef _FIX_AXIS_PLOTJUGGER_HPP
#define _FIX_AXIS_PLOTJUGGER_HPP

#include <rclcpp/rclcpp.hpp>

#include <plotjugger_msgs/msg/min_max.hpp>

namespace tools::plotjugger
{

class FixAxisPlotjugger : public rclcpp::Node
{
private:
  rclcpp::Publisher<plotjugger_msgs::msg::MinMax>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float min, max;

  void _callback();

public:
  FixAxisPlotjugger(const rclcpp::NodeOptions & options);
};

}  // namespace tools::plotjugger

#endif
