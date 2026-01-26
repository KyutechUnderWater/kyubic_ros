/**
 * @file qr_planner.cpp
 * @brief QR-code Tracking with Action Server Interface
 * @author K.Fujimoto
 * @date 2025/11/23
 *
 * @details Jetsonで取得したQRコードをもとに，トラッキングする
 ************************************************************/

#include "qr_planner/qr_planner.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <sstream>

namespace planner
{

QRPlanner::QRPlanner(const rclcpp::NodeOptions & options) : Node("qr_planner", options)
{
  reach_tolerance.x = this->declare_parameter("reach_tolerance.x", 0.1);
  reach_tolerance.y = this->declare_parameter("reach_tolerance.y", 0.1);
  reach_tolerance.z = this->declare_parameter("reach_tolerance.z", 0.1);
  reach_tolerance.roll = this->declare_parameter("reach_tolerance.roll", 0.1);
  reach_tolerance.yaw = this->declare_parameter("reach_tolerance.yaw", 0.1);

  // IPとポート設定
  udp_port = this->declare_parameter("udp_port", 22222);
  remote_ip_ = this->declare_parameter("remote_ip", "192.168.9.110");
  remote_port_ = this->declare_parameter("remote_port", 11111);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  pub_ = create_publisher<planner_msgs::msg::WrenchPlan>("goal_current_odom", qos);
  zed_power_pub_ = create_publisher<driver_msgs::msg::BoolStamped>("zed_power", qos);

  sub_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&QRPlanner::odometryCallback, this, std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<QRAction>(
    this, "qr_plan",
    std::bind(&QRPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&QRPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&QRPlanner::handle_accepted, this, std::placeholders::_1));

  // 初期化
  target_pose_.x = 0.0;
  target_pose_.y = 0.0;
  target_pose_.z = 0.0;
  target_pose_.yaw = 0.0;
  target_pose_.det_x = 0.0;
  target_pose_.det_y = 0.0;
  target_pose_.det_z = 0.0;
  target_pose_.det_conf = 0.0;
  target_pose_.z_mode = planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH;
  target_pose_.is_finished = false;

  // UDP受信スレッド開始
  stop_thread_ = false;
  sock_fd_ = -1;
  udp_thread_ = std::thread(&QRPlanner::udpReceiveThread, this);

  RCLCPP_INFO(this->get_logger(), "QR Planner (Action + UDP) started. Recv Port: %d", udp_port);
  RCLCPP_INFO(
    this->get_logger(), "QR Planner ready. Waiting for Action Goal to trigger START signal...");
}

QRPlanner::~QRPlanner()
{
  stop_thread_ = true;
  if (udp_thread_.joinable()) udp_thread_.join();
  if (sock_fd_ >= 0) close(sock_fd_);
}

// START送信の実装
void QRPlanner::sendStartSignal()
{
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create socket for sending START.");
    return;
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = inet_addr(remote_ip_.c_str());
  addr.sin_port = htons(remote_port_);

  std::string msg = "START";

  // 念のため2回送信
  sendto(sock, msg.c_str(), msg.size(), 0, (struct sockaddr *)&addr, sizeof(addr));
  usleep(1000000);  // wait wakeup(1s)
  sendto(sock, msg.c_str(), msg.size(), 0, (struct sockaddr *)&addr, sizeof(addr));

  close(sock);
  RCLCPP_INFO(
    this->get_logger(), ">>> Sent START signal to %s:%d <<<", remote_ip_.c_str(), remote_port_);
}

// UDP Thread (受信)
void QRPlanner::udpReceiveThread()
{
  sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "UDP Socket creation failed.");
    return;
  }

  struct sockaddr_in my_addr;
  memset(&my_addr, 0, sizeof(my_addr));
  my_addr.sin_family = AF_INET;
  my_addr.sin_addr.s_addr = INADDR_ANY;
  my_addr.sin_port = htons(udp_port);

  if (bind(sock_fd_, (struct sockaddr *)&my_addr, sizeof(my_addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "UDP Bind failed on port %d.", udp_port);
    close(sock_fd_);
    return;
  }

  // ノンブロッキング設定
  int flags = fcntl(sock_fd_, F_GETFL, 0);
  fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);

  char buffer[1024];
  struct sockaddr_in sender_addr;
  socklen_t addr_len = sizeof(sender_addr);
  PoseData temp_pose;

  while (!stop_thread_) {
    memset(buffer, 0, sizeof(buffer));
    ssize_t len =
      recvfrom(sock_fd_, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&sender_addr, &addr_len);

    if (len > 0) {
      std::string data_str(buffer);
      RCLCPP_INFO(this->get_logger(), "[UDP Recv] Data: %s", data_str.c_str());

      if (parse_signal_to_pose(data_str, temp_pose)) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        target_pose_ = temp_pose;

        if (target_pose_.is_finished) {
          RCLCPP_INFO(this->get_logger(), "UDP received COMPLETE signal.");
        }
      }
    }
    usleep(10000);  // 10ms wait
  }
}

bool QRPlanner::parse_signal_to_pose(const std::string & data_str, PoseData & pose)
{
  // Pythonからのフォーマット (9要素):
  // "CMD, ctrl_x, ctrl_y, ctrl_z, ctrl_yaw, det_x, det_y, det_dist, det_conf"
  try {
    std::stringstream ss(data_str);
    std::string segment;
    std::vector<std::string> parts;
    while (std::getline(ss, segment, ',')) {
      parts.push_back(segment);
    }

    std::string cmd = parts[0];

    if (cmd == "COMPLETE") {
      pose.is_finished = true;
      pose.is_error = false;
      pose.is_searching = false;
    } else if (cmd.find("ERROR") == 0 || cmd == "EXCEPTION") {
      pose.is_finished = false;
      pose.is_error = true;
      pose.is_searching = false;
    } else if (cmd == "SEARCH") {
      pose.is_finished = false;
      pose.is_error = false;
      pose.is_searching = true;
    } else {
      pose.is_finished = false;
      pose.is_error = false;
      pose.is_searching = false;
    }

    // 制御値 (Index 1-4)
    pose.x = std::stof(parts[1]);
    pose.y = std::stof(parts[2]);
    pose.z = std::stof(parts[3]);
    pose.yaw = std::stof(parts[4]);

    // 検出値 (Index 5-8)
    pose.det_x = std::stof(parts[5]);
    pose.det_y = std::stof(parts[6]);
    pose.det_z = std::stof(parts[7]);
    pose.det_conf = std::stof(parts[8]);

    pose.z_mode = planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH;

    // デバッグ: 制御値と検出値を表示
    RCLCPP_DEBUG(
      this->get_logger(), "Parsed: CMD=%s | Ctrl(X:%.1f) | Det(X:%.1f, Dist:%.2f)", cmd.c_str(),
      pose.x, pose.det_x, pose.det_z);

    return true;
  } catch (...) {
    return false;
  }
}

// Action Server Callbacks
rclcpp_action::GoalResponse QRPlanner::handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const QRAction::Goal> goal)
{
  (void)uuid;
  if (!goal->start) {
    RCLCPP_WARN(this->get_logger(), "Goal rejected: start flag is false");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "Received Goal request");
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (active_goal_handle_) {
    RCLCPP_WARN(this->get_logger(), "Goal already active. Rejecting.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse QRPlanner::handle_cancel(
  const std::shared_ptr<GoalHandleQR> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received Cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void QRPlanner::handle_accepted(const std::shared_ptr<GoalHandleQR> goal_handle)
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  active_goal_handle_ = goal_handle;

  {
    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
    target_pose_.is_finished = false;
  }

  // BoolStamped メッセージ作成 (ON)
  {
    driver_msgs::msg::BoolStamped pwr_msg;
    pwr_msg.header.stamp = this->get_clock()->now();
    pwr_msg.header.frame_id = "qr_planner";
    pwr_msg.data = true;  // ON

    zed_power_pub_->publish(pwr_msg);
    RCLCPP_INFO(this->get_logger(), "Goal Accepted. Published zed_power = TRUE");
  }

  // Wakeup Jetson
  RCLCPP_INFO(this->get_logger(), "Sending START signal to Jetson...");
  sendStartSignal();
}

// --- Main Logic ---
void QRPlanner::odometryCallback(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_odom_ = msg;
  }

  // アクティブなゴールがあるか確認
  std::shared_ptr<GoalHandleQR> goal_handle;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_handle = active_goal_handle_;
  }

  if (!goal_handle) return;

  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<QRAction::Result>();
    result->success = false;
    goal_handle->canceled(result);

    // BoolStamped メッセージ作成 (OFF)
    {
      driver_msgs::msg::BoolStamped pwr_msg;
      pwr_msg.header.stamp = this->get_clock()->now();
      pwr_msg.header.frame_id = "qr_planner";
      pwr_msg.data = false;  // OFF
      zed_power_pub_->publish(pwr_msg);
      RCLCPP_INFO(this->get_logger(), "Canceled. Published zed_power = FALSE");
    }

    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_handle_.reset();
    return;
  }

  _runPlannerLogic(goal_handle);
}

void QRPlanner::_runPlannerLogic(const std::shared_ptr<GoalHandleQR> & goal_handle)
{
  std::shared_ptr<localization_msgs::msg::Odometry> odom_copy;
  PoseData target_copy;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_copy = current_odom_;
  }
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    target_copy = target_pose_;
  }

  if (!odom_copy) return;

  // 完了時
  if (target_copy.is_finished) {
    // Zed Power OFF 送信
    {
      driver_msgs::msg::BoolStamped pwr_msg;
      pwr_msg.header.stamp = this->get_clock()->now();
      pwr_msg.header.frame_id = "qr_planner";
      pwr_msg.data = false;  // OFF
      zed_power_pub_->publish(pwr_msg);
      RCLCPP_INFO(this->get_logger(), "Finished. Published zed_power = FALSE");
    }

    auto result = std::make_shared<QRAction::Result>();
    result->success = true;
    goal_handle->succeed(result);

    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_handle_.reset();
    RCLCPP_INFO(this->get_logger(), "Action Succeeded (Received COMPLETE signal).");
    return;
  }

  // エラー時の処理
  if (target_copy.is_error) {
    RCLCPP_ERROR(
      this->get_logger(), "*** ERROR DETECTED *** Publishing Safe Pose and Aborting Action.");

    // 1. 安全座標のメッセージ作成
    auto msg = std::make_unique<planner_msgs::msg::WrenchPlan>();
    msg->header.stamp = this->get_clock()->now();
    msg->z_mode = planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH;

    msg->targets.x = odom_copy->pose.position.x;  // 0.0
    msg->targets.y = odom_copy->pose.position.y;  // 0.0
    msg->targets.z = 0.3;                         // 安全深度
    msg->targets.roll = 0.0;
    msg->targets.yaw = 0.0;  // リセット

    // 現在値などはOdomからコピー
    msg->master.x = odom_copy->pose.position.x;
    msg->master.y = odom_copy->pose.position.y;
    msg->master.z = odom_copy->pose.position.z_depth;
    msg->master.roll = odom_copy->pose.orientation.x;
    msg->master.yaw = odom_copy->pose.orientation.z;
    msg->slave.x = odom_copy->twist.linear.x;
    msg->slave.y = odom_copy->twist.linear.y;
    msg->slave.z = odom_copy->twist.linear.z_depth;
    msg->slave.roll = odom_copy->twist.angular.x;
    msg->slave.yaw = odom_copy->twist.angular.z;

    // 2. トピック送信 (安全座標へ)
    pub_->publish(std::move(msg));

    // 3. Zed Power OFF
    {
      driver_msgs::msg::BoolStamped pwr_msg;
      pwr_msg.header.stamp = this->get_clock()->now();
      pwr_msg.header.frame_id = "qr_planner";
      pwr_msg.data = false;
      zed_power_pub_->publish(pwr_msg);
    }

    // 4. アクション終了 (Aborted で返す)
    auto result = std::make_shared<QRAction::Result>();
    result->success = false;
    goal_handle->abort(result);

    // 5. リセット
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_handle_.reset();

    return;
  }

  // Feedback送信 (検出データを詰める)
  auto feedback = std::make_shared<QRAction::Feedback>();
  feedback->x = target_copy.det_x;
  feedback->y = target_copy.det_y;
  feedback->z = target_copy.det_z;
  feedback->confidence = target_copy.det_conf;
  goal_handle->publish_feedback(feedback);

  // 目標値パブリッシュ (制御指令値を詰める)
  auto msg = std::make_unique<planner_msgs::msg::WrenchPlan>();
  msg->header.stamp = this->get_clock()->now();
  msg->z_mode = planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH;

  // 通常時: 現在地(Odom) + 相対移動量(Target)
  msg->targets.x = odom_copy->pose.position.x + target_copy.x;
  msg->targets.y = odom_copy->pose.position.y + target_copy.y;
  msg->targets.z = 0.3;
  msg->targets.roll = 0.0;
  msg->targets.yaw = odom_copy->pose.orientation.z + target_copy.yaw;

  // 捜索時
  int direction = 1;
  int limit_angle = 45;
  if (target_copy.is_searching) {
    float current_angle = odom_copy->pose.orientation.z;
    // 45以上になったら「引くモード(-1)」へ切り替え
    if (current_angle >= limit_angle) {
      direction = -1;
      RCLCPP_INFO(this->get_logger(), "反時計回り");
    }
    // -45以下になったら「足すモード(1)」へ切り替え
    else if (current_angle <= -limit_angle) {
      direction = 1;
      RCLCPP_INFO(this->get_logger(), "時計回り");
    }
    // directionを掛けることで「+」と「-」を切り替える
    msg->targets.yaw = odom_copy->pose.orientation.z + (direction * target_copy.yaw);
  }
  //

  msg->master.x = odom_copy->pose.position.x;
  msg->master.y = odom_copy->pose.position.y;
  msg->master.roll = odom_copy->pose.orientation.x;
  msg->master.yaw = odom_copy->pose.orientation.z;

  msg->has_slave = true;
  msg->slave.x = odom_copy->twist.linear.x;
  msg->slave.y = odom_copy->twist.linear.y;
  msg->slave.roll = odom_copy->twist.angular.x;
  msg->slave.yaw = odom_copy->twist.angular.z;

  if (target_copy.z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH) {
    msg->master.z = odom_copy->pose.position.z_depth;
    msg->slave.z = odom_copy->twist.linear.z_depth;
  } else {
    msg->master.z = odom_copy->pose.position.z_altitude;
    msg->slave.z = odom_copy->twist.linear.z_altitude;
  }

  pub_->publish(std::move(msg));
}

bool QRPlanner::_checkReached(const PoseData & target, const Tolerance & tolerance)
{
  (void)target;
  (void)tolerance;
  return false;
}

}  // namespace planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planner::QRPlanner)
