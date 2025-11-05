/**
 * @file fix_axis_plotjuggler.hpp
 * @brief publish min and max value for fixed axis range of plotjuggler
 * @author R.Ohnishi
 * @date 2025/06/15
 *
 * @details plotjugglerのグラフの軸範囲を指定するためのPublisher
 ********************************************************************/

#ifndef _FIX_AXIS_plotjuggler_HPP
#define _FIX_AXIS_plotjuggler_HPP

#include <plotjuggler_msgs/msg/min_max.hpp>
#include <rclcpp/rclcpp.hpp>

namespace tools::plotjuggler
{

class FixAxisPlotjuggler : public rclcpp::Node
{
private:
  rclcpp::Publisher<plotjuggler_msgs::msg::MinMax>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float min, max;

  void _callback();

public:
  FixAxisPlotjuggler(const rclcpp::NodeOptions & options);
};

}  // namespace tools::plotjuggler

#endif
