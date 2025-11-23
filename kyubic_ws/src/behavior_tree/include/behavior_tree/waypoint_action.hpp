#ifndef _WAYPOINT_ACTION_HPP
#define _WAYPOINT_ACTION_HPP

#include <planner_msgs/action/pdla.hpp>
// [修正] 自作の抽象クラスをインクルード
// ※このファイルのパスは実際の配置場所に合わせる必要があります
#include "behavior_tree/common/ros_action_node.hpp"

namespace behavior_tree
{

/**
 * @brief Waypoint Action Client implementation using the CUSTOM RosActionNode template.
 */
class WaypointAction : public RosActionNode<planner_msgs::action::PDLA>
{
public:
  /**
   * @brief Constructor matching the custom base class signature
   */
  WaypointAction(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  static BT::PortsList providedPorts();

  bool setGoal(typename Action::Goal & goal) override;

  BT::NodeStatus onResult(const WrappedResult & wr) override;

  void onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};

}  // namespace behavior_tree

#endif  // _WAYPOINT_ACTION_HPP