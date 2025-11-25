/**
 * @file qr_action.hpp
 * @brief Action client for QR tracking
 * @author S.Itozono
 * @date 2025/11/22
 *
 * @details QRコードトラッキングのAction ClientのBTノード
 *******************************************************/

#ifndef _QR_ACTION_HPP
#define _QR_ACTION_HPP

#include <planner_msgs/action/qr.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "behavior_tree/common/ros_action_node.hpp"

namespace behavior_tree
{

/**
 * @brief QR Action Client implementation using the CUSTOM RosActionNode template.
 * Target Action: planner_msgs::action::QR
 */
class QrAction : public RosActionNode<planner_msgs::action::QR>
{
public:
  /**
   * @brief Constructor matching the custom base class signature
   */
  QrAction(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub,
    rclcpp::Node::SharedPtr ros_node);

  /**
   * @brief Define ports
   * Input: action_name
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Sets the goal.start to true to trigger the server.
   */
  bool setGoal(typename Action::Goal & goal) override;

  /**
   * @brief Checks if the action finished successfully.
   */
  BT::NodeStatus onResult(const WrappedResult & wr) override;

  /**
   * @brief Feedback callback (Empty for QR action but required to implement)
   */
  void onFeedback(const std::shared_ptr<const Feedback> feedback) override;

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub_;
};

}  // namespace behavior_tree

#endif  // _QR_ACTION_HPP
