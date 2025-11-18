/**
 * @file ros_action_node.hpp
 * @brief Abstract class for ROS2 Action client with Behavior Tree
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details BehaviorTreeでROS2のAction通信をするクライアントの抽象クラス
 **********************************************************************/

#ifndef _ROS_ACTION_NODE_HPP
#define _ROS_ACTION_NODE_HPP

#include <behaviortree_cpp/action_node.h>

#include <future>
#include <rclcpp_action/rclcpp_action.hpp>

namespace behavior_tree
{

/**
 * @brief Template for ROS2 Action with Behavior Tree
 * @tparam ActionT Action Type
 * @details behaviortree_ros2パッケージが動かなかったので，自作
 */
template <class ActionT>
class RosActionNode : public BT::StatefulActionNode
{
public:
  using Action = ActionT;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using ActionClient = rclcpp_action::Client<Action>;
  using WrappedResult = typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult;
  using Feedback = typename Action::Feedback;

  /**
   * @brief Create action client
   * @param name Node name
   * @param config Node config
   * @param ros_node pointer of ros node
   * @details Get action_name from xml, and then create action client
   */
  RosActionNode(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  /**
   * @brief Configures the Goal message
   * @param goal The Goal object to be sent to the server
   * @return true if the goal was set successfully, false otherwise (triggers FAILURE)
   */
  virtual bool setGoal(typename Action::Goal & goal) = 0;

  /**
   * @brief Processes the final result received from the server
   * @param wr The wrapped result containing the result code and data
   * @return BT::NodeStatus (typically SUCCESS or FAILURE)
   */
  virtual BT::NodeStatus onResult(const WrappedResult & wr) = 0;

  /**
   * @brief Behavior when recieving feadback
   */
  virtual void onFeedback(const std::shared_ptr<const Feedback> feedback);

  /**
   * @brief Called when the node is first ticked. Initiates the ROS 2 action.
   * @return BT::NodeStatus::RUNNING if the goal is sent successfully,
   *         BT::NodeStatus::FAILURE if the server is missing or goal setting fails.
   * @details Checks server availability, configures the goal using the derived class's setGoal,
   *          and sends the goal asynchronously.
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief This method manages the asynchronous state machine:
   * @return BT::NodeStatus::RUNNING while waiting for the server.
   *         Returns the result of onResult() once the action completes.
   * @details 1. Checks if the goal was accepted by the server.
   *          2. Checks if the result has been received.
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Cancels the currently active goal if one exists.
   */
  void onHalted() override;

protected:
  rclcpp::Node::SharedPtr ros_node_;
  typename ActionClient::SharedPtr client_;
  std::string action_name;

  std::shared_future<typename GoalHandle::SharedPtr> future_goal_handle_;
  std::shared_future<WrappedResult> future_result_;
  typename GoalHandle::SharedPtr active_goal_handle_;
};

// =================================================================================
// Implementation
// =================================================================================

template <class ActionT>
RosActionNode<ActionT>::RosActionNode(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::StatefulActionNode(name, config), ros_node_(ros_node)
{
  if (!this->getInput("action_name", action_name)) {
    action_name = this->registrationName();
  }

  client_ = rclcpp_action::create_client<Action>(ros_node_, action_name);
}

template <class ActionT>
void RosActionNode<ActionT>::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  // Do nothing by default
  (void)feedback;
}

template <class ActionT>
BT::NodeStatus RosActionNode<ActionT>::onStart()
{
  if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Action server '%s' not found", action_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Call setGoal() of the derived class
  typename Action::Goal goal_msg;
  if (!setGoal(goal_msg)) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Failed to configure the Goal");
    return BT::NodeStatus::FAILURE;
  }

  auto send_goal_options = typename ActionClient::SendGoalOptions();
  send_goal_options.feedback_callback =
    [this](typename GoalHandle::SharedPtr, const std::shared_ptr<const Feedback> feedback) {
      this->onFeedback(feedback);
    };

  future_goal_handle_ = client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(ros_node_->get_logger(), "Sending goal: %s", action_name.c_str());
  return BT::NodeStatus::RUNNING;
}

template <class ActionT>
BT::NodeStatus RosActionNode<ActionT>::onRunning()
{
  // 1. Check if the goal was accepted
  if (!active_goal_handle_) {
    if (future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      active_goal_handle_ = future_goal_handle_.get();
      if (!active_goal_handle_) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Goal was rejected by the server");
        return BT::NodeStatus::FAILURE;
      }

      RCLCPP_INFO(ros_node_->get_logger(), "Goal accepted. Waiting for result.");
      future_result_ = client_->async_get_result(active_goal_handle_);
      return BT::NodeStatus::RUNNING;
    } else {
      return BT::NodeStatus::RUNNING;
    }
  }

  // 2. Check for the result
  if (future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    WrappedResult wr = future_result_.get();
    active_goal_handle_.reset();

    // Call onResult() of the derived class
    return onResult(wr);

  } else {
    return BT::NodeStatus::RUNNING;
  }
}

template <class ActionT>
void RosActionNode<ActionT>::onHalted()
{
  RCLCPP_WARN(ros_node_->get_logger(), "Action was halted");
  if (active_goal_handle_) {
    client_->async_cancel_goal(active_goal_handle_);
    active_goal_handle_.reset();
  }
}

}  // namespace behavior_tree

#endif  // !_ROS_ACTION_NODE_HPP
