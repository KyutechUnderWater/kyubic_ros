/**
 * @file qr_planner.hpp
 * @brief UDP Goal Tracking with Action Server Interface
 */

#ifndef QR_PLANNER_HPP
#define QR_PLANNER_HPP

#include <driver_msgs/msg/bool_stamped.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>  // 追加: Zed Power用
// アクション定義 (パッケージ名は環境に合わせてください。ここでは planner_msgs と仮定)
#include <mutex>
#include <planner_msgs/action/qr.hpp>
#include <string>
#include <thread>
#include <vector>

namespace planner
{

// アクション型のエイリアス
using QRAction = planner_msgs::action::QR;
using GoalHandleQR = rclcpp_action::ServerGoalHandle<QRAction>;

struct PoseData
{
  // 制御指令値 (Python: ctrl_*)
  float x;
  float y;
  float z;
  float roll;
  float yaw;
  uint8_t z_mode;
  bool is_finished;
  bool is_error = false;  // ★★★ 追加 ★★★
  // ★★★ 追加: 検出データ (Python: det_*) ★★★
  float det_x;     // 画像上のX座標
  float det_y;     // 画像上のY座標
  float det_z;     // 距離 (dist)
  float det_conf;  // 信頼度
};

struct Tolerance
{
  float x;
  float y;
  float z;
  float roll;
  float yaw;
};

class QRPlanner : public rclcpp::Node
{
public:
  explicit QRPlanner(const rclcpp::NodeOptions & options);
  ~QRPlanner();

private:
  // コールバック
  void odometryCallback(const localization_msgs::msg::Odometry::SharedPtr msg);

  // アクションサーバー用コールバック
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const QRAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleQR> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleQR> goal_handle);

  // ロジック
  void _runPlannerLogic(const std::shared_ptr<GoalHandleQR> & goal_handle);
  bool _checkReached(const PoseData & target, const Tolerance & tolerance);

  // UDP関連
  void udpReceiveThread();
  void sendStartSignal();
  bool parse_signal_to_pose(const std::string & data_str, PoseData & pose);

  // 変数
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<planner_msgs::msg::WrenchPlan>::SharedPtr pub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_;

  // ★★★ 追加: Zed Power Publisher ★★★
  // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zed_power_pub_;
  rclcpp::Publisher<driver_msgs::msg::BoolStamped>::SharedPtr zed_power_pub_;
  rclcpp_action::Server<QRAction>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleQR> active_goal_handle_;

  localization_msgs::msg::Odometry::SharedPtr current_odom_;
  PoseData target_pose_;
  Tolerance reach_tolerance;

  std::mutex odom_mutex_;
  std::mutex pose_mutex_;
  std::mutex goal_mutex_;

  // UDP設定
  std::string remote_ip_;
  int remote_port_;
  int sock_fd_;
  std::thread udp_thread_;
  bool stop_thread_;
};

}  // namespace planner

#endif