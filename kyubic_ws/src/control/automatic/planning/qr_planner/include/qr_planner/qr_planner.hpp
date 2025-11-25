/**
 * @file qr_planner.hpp
 * @brief QR-code Tracking with Action Server Interface
 * @author K.Fujimoto
 * @date 2025/11/23
 *
 * @details Jetsonで取得したQRコードをもとに，トラッキングする
 ************************************************************/

#ifndef _QR_PLANNER_HPP
#define _QR_PLANNER_HPP

#include <driver_msgs/msg/bool_stamped.hpp>
#include <localization_msgs/msg/odometry.hpp>
#include <mutex>
#include <planner_msgs/action/qr.hpp>
#include <planner_msgs/msg/wrench_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace planner
{

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
  bool is_error = false;
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
  // Callback
  void odometryCallback(const localization_msgs::msg::Odometry::SharedPtr msg);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const QRAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleQR> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleQR> goal_handle);

  // Logic
  void _runPlannerLogic(const std::shared_ptr<GoalHandleQR> & goal_handle);
  bool _checkReached(const PoseData & target, const Tolerance & tolerance);

  // UDP
  void udpReceiveThread();
  void sendStartSignal();
  bool parse_signal_to_pose(const std::string & data_str, PoseData & pose);

  // Valiables
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<planner_msgs::msg::WrenchPlan>::SharedPtr pub_;
  rclcpp::Publisher<driver_msgs::msg::BoolStamped>::SharedPtr zed_power_pub_;
  rclcpp::Subscription<localization_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp_action::Server<QRAction>::SharedPtr action_server_;

  std::shared_ptr<GoalHandleQR> active_goal_handle_;
  localization_msgs::msg::Odometry::SharedPtr current_odom_;

  PoseData target_pose_;
  Tolerance reach_tolerance;

  std::mutex odom_mutex_;
  std::mutex pose_mutex_;
  std::mutex goal_mutex_;

  // UDP config
  std::string remote_ip_;
  int remote_port_;
  uint16_t udp_port;
  int sock_fd_;
  std::thread udp_thread_;
  bool stop_thread_;
};

}  // namespace planner

#endif
