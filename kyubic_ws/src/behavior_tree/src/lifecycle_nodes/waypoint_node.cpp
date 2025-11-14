#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// 既存のAction定義ファイルをインクルード
#include <memory>
#include <string>

#include "projection_dynamic_look_ahead_planner/action/pdla.hpp"

namespace robot_controller
{

// PDLA Actionの型定義
using PDLA = projection_dynamic_look_ahead_planner::action::PDLA;
using GoalHandlePDLA = rclcpp_action::ClientGoalHandle<PDLA>;

class WaypointNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit WaypointNode(const rclcpp::NodeOptions & options)
  : LifecycleNode("waypointNode", options)
  {
    RCLCPP_INFO(get_logger(), "WaypointNode (Action Client) is created.");
  }

  // 1. Configure (設定)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring WaypointNode...");

    // パラメータから実行すべきCSVパスを宣言・取得
    try {
      this->declare_parameter<std::string>("csv_path", "");
      csv_path_ = this->get_parameter("csv_path").as_string();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to declare/get 'csv_path' parameter: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    if (csv_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "'csv_path' parameter is empty. Cannot configure.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // PDLA Action Server ("pdla_plan") のためのAction Clientを作成
    action_client_ = rclcpp_action::create_client<PDLA>(
      this,
      "pdla_plan");  // <-- PDLA Action Server名

    RCLCPP_INFO(get_logger(), "WaypointNode configured. CSV path: %s", csv_path_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 2. Activate (起動)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating WaypointNode...");

    if (!action_client_) {
      RCLCPP_ERROR(get_logger(), "Action client is not initialized.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Action Serverが起動するまで待機
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "PDLA Action Server 'pdla_plan' not available.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // Goal (CSVパス) を作成
    auto goal_msg = PDLA::Goal();
    goal_msg.csv_file_path = csv_path_;

    // Goalを非同期で送信
    auto send_goal_options = rclcpp_action::Client<PDLA>::SendGoalOptions();

    // Goalが受理されたときのコールバック
    send_goal_options.goal_response_callback =
      [this](const GoalHandlePDLA::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, starting waypoint following.");
          // 有効なGoal Handleを保持
          std::lock_guard<std::mutex> lock(goal_mutex_);
          this->goal_handle_ = goal_handle;
        }
      };

    // (オプション) Feedbackコールバック
    send_goal_options.feedback_callback =
      [](GoalHandlePDLA::SharedPtr, const std::shared_ptr<const PDLA::Feedback> feedback) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("WaypointNode"), "Feedback: Waypoint index %u",
          feedback->current_waypoint_index);
      };

    // (オプション) Resultコールバック
    send_goal_options.result_callback = [](const GoalHandlePDLA::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(rclcpp::get_logger("WaypointNode"), "Waypoint following SUCCEEDED");
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("WaypointNode"), "Waypoint following failed (code: %d)",
          static_cast<int>(result.code));
      }
    };

    RCLCPP_INFO(this->get_logger(), "Sending goal to PDLA Action Server...");
    action_client_->async_send_goal(goal_msg, send_goal_options);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 3. Deactivate (停止)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Deactivating WaypointNode...");

    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (goal_handle_) {
      RCLCPP_INFO(get_logger(), "Canceling current waypoint goal...");
      action_client_->async_cancel_goal(goal_handle_);
      goal_handle_.reset();
    } else {
      RCLCPP_INFO(get_logger(), "No active goal to cancel.");
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 4. Cleanup (クリーンアップ)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up WaypointNode...");
    action_client_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 5. Shutdown (シャットダウン)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Shutting down WaypointNode...");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  rclcpp_action::ActionClient<PDLA>::SharedPtr action_client_;
  GoalHandlePDLA::SharedPtr goal_handle_;
  std::string csv_path_;
  std::mutex goal_mutex_;
};
}  // namespace robot_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_controller::WaypointNode)