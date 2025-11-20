/**
 * @file qr_planner.hpp
 * @brief UDP Goal Tracking with Action Server Interface
 * @author R.Ohnishi (Modified)
 * @date 2025/11/19
 ****************************************************************************/

#ifndef _QR_PLANNER_HPP
#define _QR_PLANNER_HPP

#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <planner_msgs/action/qr.hpp>
#include <localization_msgs/msg/odometry.hpp>

// // ★★★ 自パッケージのアクションヘッダ (ビルド時に自動生成される) ★★★
// #include "qr_planner/action/qr.hpp"

namespace planner
{

// ★★★ これがないとエラーになる (型エイリアス) ★★★
using QRAction = planner_msgs::action::QR; 
using GoalHandleQR = rclcpp_action::ServerGoalHandle<QRAction>;

struct PoseData {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double yaw = 0.0;
  uint8_t z_mode = 0;
  bool is_finished = false;
};

struct Tolerance {
  double x; double y; double z; double roll; double yaw;
};

class QRPlanner : public rclcpp::Node
{
public:
  explicit QRPlanner(const rclcpp::NodeOptions & options);
  ~QRPlanner();

private:
  // UDP
  void udpReceiveThread();
  bool parse_signal_to_pose(const std::string& data_str, PoseData& pose);
  void sendStartSignal();

  std::thread udp_thread_;
  std::atomic<bool> stop_thread_{false};
  int sock_fd_;
  std::string remote_ip_;
  int remote_port_;

  // ★★★ Action Server (型を QRAction に統一) ★★★
  rclcpp_action::Server<QRAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const QRAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleQR> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleQR> goal_handle);

  // Logic
  void odometryCallback(const localization_msgs::msg::Odometry::SharedPtr msg);
  
  // ★★★ 引数の型を修正 (PDLA -> GoalHandleQR) ★★★
  void _runPlannerLogic(const std::shared_ptr<GoalHandleQR> & goal_handle);

  bool _checkReached(const PoseData & target, const Tolerance & tolerance);

  // Variables
  Tolerance reach_tolerance;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<planner_msgs::msg::WrenchPlan>::SharedPtr pub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_;

  std::mutex odom_mutex_;
  localization_msgs::msg::Odometry::SharedPtr current_odom_;

  std::mutex goal_mutex_;
  std::shared_ptr<GoalHandleQR> active_goal_handle_;

  std::mutex pose_mutex_;
  PoseData target_pose_;
};

}  // namespace planner

#endif  // !_QR_PLANNER_HPP