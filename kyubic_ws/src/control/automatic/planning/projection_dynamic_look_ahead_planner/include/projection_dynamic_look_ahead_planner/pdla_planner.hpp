/**
 * @file PDLA_planner.hpp
 * @brief Tracking through the pass with Projection Dynamic Look-Ahead Planner
 * @author R.Ohnishi
 * @date 2025/05/13
 *
 * @details ベクトル射影を用いた動的前方注視制御による経路追従
 ****************************************************************************/

#ifndef _PLDA_PLANNER_HPP
#define _PLDA_PLANNER_HPP

#include <path_planner/path_csv_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <localization_msgs/msg/odometry.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>

#include <sys/types.h>

namespace planner
{

struct Tolerance
{
  double x;
  double y;
  double z;
  double roll;
  double yaw;
};

class PDLAPlanner : public rclcpp::Node
{
private:
  rclcpp::Publisher<planner_msgs::msg::WrenchPlan>::SharedPtr pub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_;

  std::vector<PoseData> target_pose_;
  std::shared_ptr<localization_msgs::msg::Odometry> current_odom_;

  double look_ahead_scale;
  Tolerance reach_tolerance, waypoint_tolerance;

  bool catmull_rom = false;
  uint step_idx = 0;

  void odometryCallback(const localization_msgs::msg::Odometry::SharedPtr msg);

  void _updateGoal();
  bool _checkReached(PoseData & target_pose, Tolerance tolerance);

  void _print_waypoint(std::string label, size_t step_idx);

public:
  explicit PDLAPlanner(const rclcpp::NodeOptions & options);
};

}  // namespace planner

#endif  // !_PLDA_PLANNER_HPP
