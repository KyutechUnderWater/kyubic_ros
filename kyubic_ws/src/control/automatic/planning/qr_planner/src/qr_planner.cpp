/**
 * @file qr_planner.cpp
 * @brief UDP Goal Tracking with Action Server Interface
 * @author R.Ohnishi (Modified)
 * @date 2025/11/19
 ****************************************************************************/

#include "qr_planner/qr_planner.hpp"

#include <Eigen/Dense>
#include <sstream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

namespace planner
{

// Jetson側の受信ポート (techno.pyの送信先)
const int UDP_PORT = 22222;

QRPlanner::QRPlanner(const rclcpp::NodeOptions & options) : Node("qr_planner", options)
{
  // パラメータ
  reach_tolerance.x = this->declare_parameter("reach_tolerance.x", 0.1);
  reach_tolerance.y = this->declare_parameter("reach_tolerance.y", 0.1);
  reach_tolerance.z = this->declare_parameter("reach_tolerance.z", 0.1);
  reach_tolerance.roll = this->declare_parameter("reach_tolerance.roll", 0.1);
  reach_tolerance.yaw = this->declare_parameter("reach_tolerance.yaw", 0.1);

  // ★★★ 追加: Techno.py (PC) のIPとポート設定 ★★★
  // デフォルトは適当な値なので、起動時にパラメータでPCのIPを指定してください
  remote_ip_ = this->declare_parameter("remote_ip", "192.168.9.245"); 
  remote_port_ = this->declare_parameter("remote_port", 11111); // techno.pyのMY_PORT

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  pub_ = create_publisher<planner_msgs::msg::WrenchPlan>("goal_current_odom", qos);
  sub_ = create_subscription<localization_msgs::msg::Odometry>(
    "odom", qos, std::bind(&QRPlanner::odometryCallback, this, std::placeholders::_1));

// ★★★ Action Server 作成 (QR型に変更) ★★★
  action_server_ = rclcpp_action::create_server<QRAction>(
    this,
    "qr_action",  // アクション名もシンプルにした
    std::bind(&QRPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&QRPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&QRPlanner::handle_accepted, this, std::placeholders::_1)
  );

  // 初期化
  target_pose_.x = 0.0;
  target_pose_.z_mode = planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH;
  target_pose_.is_finished = false;

  // UDP受信スレッド開始
  stop_thread_ = false;
  udp_thread_ = std::thread(&QRPlanner::udpReceiveThread, this);

  // ★★★起動時にSTART信号を送信 ★★★
  // sendStartSignal();

  RCLCPP_INFO(this->get_logger(), "QR Planner (Action + UDP) started. Recv Port: %d", UDP_PORT);
  RCLCPP_INFO(this->get_logger(), "QR Planner ready. Waiting for Action Goal to trigger START signal...");
}

QRPlanner::~QRPlanner()
{
  stop_thread_ = true;
  if (udp_thread_.joinable()) udp_thread_.join();
  if (sock_fd_ >= 0) close(sock_fd_);
}

// ★★★ 追加: START送信の実装 ★★★
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
    usleep(100000); // 0.1s wait
    sendto(sock, msg.c_str(), msg.size(), 0, (struct sockaddr *)&addr, sizeof(addr));

    close(sock);
    RCLCPP_INFO(this->get_logger(), ">>> Sent START signal to %s:%d <<<", remote_ip_.c_str(), remote_port_);
}

// --- UDP Thread (受信) ---
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
  my_addr.sin_port = htons(UDP_PORT);

  if (bind(sock_fd_, (struct sockaddr*)&my_addr, sizeof(my_addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "UDP Bind failed on port %d.", UDP_PORT);
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
    ssize_t len = recvfrom(sock_fd_, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&sender_addr, &addr_len);

    if (len > 0) {
      std::string data_str(buffer);
      // ★★★ 追加: 受信した生データを表示 ★★★
      // ※ 高頻度で来るなら RCLCPP_INFO_THROTTLE を使うべきだが、今は全件見る
      RCLCPP_INFO(this->get_logger(), "[UDP Recv] Addr: %s | Data: %s", 
      inet_ntoa(sender_addr.sin_addr), data_str.c_str());

      if (parse_signal_to_pose(data_str, temp_pose)) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        target_pose_ = temp_pose;
        
        if (target_pose_.is_finished) {
            RCLCPP_INFO(this->get_logger(), "UDP received COMPLETE signal.");
        }
      }
    }
    usleep(10000); // 10ms wait
  }
}

bool QRPlanner::parse_signal_to_pose(const std::string& data_str, PoseData& pose)
{
  // Format: "CMD,x,y,z,yaw"
  try {
    std::stringstream ss(data_str);
    std::string segment;
    std::vector<std::string> parts;
    while(std::getline(ss, segment, ',')) {
        parts.push_back(segment);
    }

    if (parts.size() != 5) return false;

    std::string cmd = parts[0];
    
    // 
    if (cmd == "COMPLETE") {
      pose.is_finished = true;
    } else {
      pose.is_finished = false;
    }

    pose.x = std::stof(parts[1]);
    pose.y = std::stof(parts[2]);
    pose.z = std::stof(parts[3]);
    pose.yaw = std::stof(parts[4]);
    pose.z_mode = planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH;
    
    //今回は全部出す
    RCLCPP_INFO(this->get_logger(), "Parsed: CMD=%s -> X:%.2f Y:%.2f Z:%.2f Yaw:%.2f", 
       cmd.c_str(), pose.x, pose.y, pose.z, pose.yaw);

    return true;
  } catch (...) {
    return false;
  }
}

// --- Action Server Callbacks ---
rclcpp_action::GoalResponse QRPlanner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const QRAction::Goal> goal)
{
  (void)uuid;
  // goal->start が true かチェックしてもいい
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

void QRPlanner::handle_accepted(
  const std::shared_ptr<GoalHandleQR> goal_handle)
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  active_goal_handle_ = goal_handle;
  // 目標が来たらUDP受信データのフラグをリセットするなど
  {
      std::lock_guard<std::mutex> pose_lock(pose_mutex_);
      target_pose_.is_finished = false;
  }
  // ★★★ ここでJetsonを起こす！ ★★★
  RCLCPP_INFO(this->get_logger(), "Goal Accepted. Sending START signal to Jetson...");
  sendStartSignal();
}

// --- Main Logic ---!!!

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
  
  // ゴールがないなら何もしない
  if (!goal_handle) return; 

  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<QRAction::Result>();
    result->success = false;
    goal_handle->canceled(result);
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_handle_.reset();
    return;
  }

  _runPlannerLogic(goal_handle);
}

void QRPlanner::_runPlannerLogic(const std::shared_ptr<GoalHandleQR> & goal_handle)
{
  // ... (odom_copy, target_copy の取得ロジックはそのまま) ...
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
    // ★★★ PDLA::Result ではなく QRAction::Result を使う ★★★
    auto result = std::make_shared<QRAction::Result>();
    result->success = true;
    goal_handle->succeed(result);

    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_handle_.reset();
    RCLCPP_INFO(this->get_logger(), "Action Succeeded (Received COMPLETE signal).");
    return;
  }

  auto msg = std::make_unique<planner_msgs::msg::WrenchPlan>();
  msg->header.stamp = this->get_clock()->now();
  msg->z_mode = target_copy.z_mode;

  msg->targets.x = target_copy.x;
  msg->targets.y = target_copy.y;
  msg->targets.z = target_copy.z;
  msg->targets.yaw = target_copy.yaw;
  msg->targets.roll = 0.0;

  msg->master.x = odom_copy->pose.position.x;
  msg->master.y = odom_copy->pose.position.y;
  msg->master.roll = odom_copy->pose.orientation.x;
  msg->master.yaw = odom_copy->pose.orientation.z;

  if (target_copy.z_mode == planner_msgs::msg::WrenchPlan::Z_MODE_DEPTH) {
    msg->master.z = odom_copy->pose.position.z_depth;
    msg->slave.z = odom_copy->twist.linear.z_depth;
  } else {
    msg->master.z = odom_copy->pose.position.z_altitude;
    msg->slave.z = odom_copy->twist.linear.z_altitude;
  }

  msg->slave.x = odom_copy->twist.linear.x;
  msg->slave.y = odom_copy->twist.linear.y;
  msg->slave.roll = odom_copy->twist.angular.x;
  msg->slave.yaw = odom_copy->twist.angular.z;

  pub_->publish(std::move(msg));

  if (target_copy.is_finished) {
      auto result = std::make_shared<QRAction::Result>();
      result->success = true;
      goal_handle->succeed(result);
      
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_handle_.reset();
      RCLCPP_INFO(this->get_logger(), "Action Succeeded.");
  }

  // auto feedback = std::make_shared<planner_msgs::action::PDLA::Feedback>();
  // feedback->current_odom = *odom_copy;
  // feedback->current_waypoint_index = 0;
  // goal_handle->publish_feedback(feedback);
}

bool QRPlanner::_checkReached(const PoseData & target, const Tolerance & tolerance)
{
  (void)target;    // ★ 追加
  (void)tolerance; // ★ 追加
  return false; 
}

} // namespace planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planner::QRPlanner)