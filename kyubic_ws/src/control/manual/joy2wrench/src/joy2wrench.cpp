#include "joy2wrench/joy2wrench.hpp"
#include <cmath> // sin, cos, atan2のため
#include <iostream>

namespace joy2wrench
{
<<<<<<< Updated upstream
Joy2WrenchStamped::Joy2WrenchStamped() : Node("joy_to_wrench_stamped")
{
  device_name = this->declare_parameter("device_name", "");
  force_x_scale = this->declare_parameter("force_x_scale", 1.0);
  force_y_scale = this->declare_parameter("force_y_scale", 1.0);
  force_z_scale = this->declare_parameter("force_z_scale", 1.0);
  torque_x_scale = this->declare_parameter("torque_x_scale", 1.0);
  torque_z_scale = this->declare_parameter("torque_z_scale", 1.0);
=======
>>>>>>> Stashed changes

Joy2WrenchStamped::Joy2WrenchStamped() : Node("joy2wrench")
{
  // パラメータの宣言
  device_name = this->declare_parameter("device_name", "");
  force_x_scale = this->declare_parameter("force_x_scale", 1.0);
  force_y_scale = this->declare_parameter("force_y_scale", 1.0);
  force_z_scale = this->declare_parameter("force_z_scale", 1.0);
  torque_x_scale = this->declare_parameter("torque_x_scale", 1.0);
  torque_z_scale = this->declare_parameter("torque_z_scale", 1.0);
  auto odom_topic = this->declare_parameter("odom_topic", "/odom");
  // 起動時のデフォルトモードをパラメータで設定
  is_absolute_mode_ = this->declare_parameter("default_mode_is_absolute", false);

  // PublisherとJoy Subscriberの作成
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("robot_force", qos);
  sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&Joy2WrenchStamped::_joyCallback, this, std::placeholders::_1));

  // Odometry Subscriberの作成
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&Joy2WrenchStamped::_odomCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), "Joy to Wrench node has been started.");
  RCLCPP_INFO(this->get_logger(), "Initial mode: %s", is_absolute_mode_ ? "Absolute" : "Robot");
}

// Odometryメッセージを受け取るたびに、ロボットの方位角を更新する
void Joy2WrenchStamped::_odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto& q = msg->pose.pose.orientation;
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  std::lock_guard<std::mutex> lock(yaw_mutex_);
  current_yaw_ = yaw;
}

// Joyメッセージを受け取るたびに、力を計算して配信する
void Joy2WrenchStamped::_joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
<<<<<<< Updated upstream
  auto wrench_msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
  wrench_msg->header = msg->header;

  // TODO: PS3有線追加
  if (device_name == "PLAYSTATION(R)3 Controller") {
    wrench_msg->wrench.force.x = msg->axes[1] * force_x_scale;
    wrench_msg->wrench.force.y = msg->axes[0] * -force_y_scale;
    wrench_msg->wrench.force.z = msg->axes[4] * -force_z_scale;

    if (msg->axes[2] < 1 && msg->axes[5]) {
      wrench_msg->wrench.torque.x = -(msg->axes[2] - 1) * -torque_x_scale;
    } else if (msg->axes[2] && msg->axes[5] < 1) {
      wrench_msg->wrench.torque.x = -(msg->axes[5] - 1) * torque_x_scale;
    } else {
      wrench_msg->wrench.torque.x = 0.0;
    }

    wrench_msg->wrench.torque.z = msg->axes[3] * -torque_z_scale;
  } else if (device_name == "Logicool Dual Action") {
    wrench_msg->wrench.force.x = msg->axes[1] * force_x_scale;
    wrench_msg->wrench.force.y = msg->axes[0] * -force_y_scale;
    wrench_msg->wrench.force.z = msg->axes[3] * -force_z_scale;

    if (msg->buttons[6] && !msg->buttons[7]) {
      wrench_msg->wrench.torque.x = -torque_x_scale;
    } else if (!msg->buttons[6] && msg->buttons[7]) {
      wrench_msg->wrench.torque.x = torque_x_scale;
    } else {
      wrench_msg->wrench.torque.x = 0.0;
    }

    wrench_msg->wrench.torque.z = msg->axes[2] * -torque_z_scale;
  } else {
    std::cerr << "Not found device: " << device_name << std::endl;
  }
  wrench_msg->wrench.torque.y = 0.0;
=======
  // --- ▼▼▼ モード切替処理を追加 ▼▼▼ ---
  bool current_button_state = (msg->buttons.size() > 9 && msg->buttons[9] == 1);
  // ボタンが押された瞬間（立ち上がりエッジ）を検出
  if (current_button_state && !prev_button_state_) {
    //合図のログ出力を追加
    RCLCPP_INFO(this->get_logger(), "Button 9 Pressed: Toggling mode!");
    is_absolute_mode_ = !is_absolute_mode_; // モードをトグル
    RCLCPP_INFO(this->get_logger(), "Mode switched to: %s", is_absolute_mode_ ? "Absolute" : "Robot");
  }
  prev_button_state_ = current_button_state;
  // --- ▲▲▲ ここまで追加 ▲▲▲ ---

  // STEP 1: ロボット基準の力・トルクを計算
  geometry_msgs::msg::Vector3 force_robot;
  geometry_msgs::msg::Vector3 torque_robot;
  
  if (device_name == "Logicool Dual Action") {
    force_robot.x = msg->axes[1] * force_x_scale;
    force_robot.y = msg->axes[0] * -force_y_scale;
    force_robot.z = msg->axes[3] * -force_z_scale;
    if (msg->buttons[6] && !msg->buttons[7]) { torque_robot.x = -torque_x_scale; }
    else if (!msg->buttons[6] && msg->buttons[7]) { torque_robot.x = torque_x_scale; }
    else { torque_robot.x = 0.0; }
    torque_robot.z = msg->axes[2] * -torque_z_scale;
  } else if (device_name == "PLAYSTATION(R)3 Controller") {
    force_robot.x = msg->axes[1] * force_x_scale;
    force_robot.y = msg->axes[0] * -force_y_scale;
    force_robot.z = msg->axes[4] * -force_z_scale;
    if (msg->axes[2] < 1 && msg->axes[5]) { torque_robot.x = -(msg->axes[2] - 1) * -torque_x_scale; }
    else if (msg->axes[2] && msg->axes[5] < 1) { torque_robot.x = -(msg->axes[5] - 1) * torque_x_scale; }
    else { torque_robot.x = 0.0; }
    torque_robot.z = msg->axes[3] * -torque_z_scale;
  } else {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Not found device: %s", device_name.c_str());
    return;
  }
  torque_robot.y = 0.0;

  // メッセージの準備
  auto wrench_msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
  wrench_msg->header.stamp = this->get_clock()->now();

  // --- ▼▼▼ モードに応じて処理を分岐 ▼▼▼ ---
  if (is_absolute_mode_) {
    // 【絶対座標モード】
    double yaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(yaw_mutex_);
      yaw = current_yaw_;
    }
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    // 回転行列を使って手動で座標変換
    wrench_msg->wrench.force.x = force_robot.x * cos_yaw - force_robot.y * sin_yaw;
    wrench_msg->wrench.force.y = force_robot.x * sin_yaw + force_robot.y * cos_yaw;
    wrench_msg->wrench.force.z = force_robot.z;

    wrench_msg->wrench.torque.x = torque_robot.x * cos_yaw - torque_robot.y * sin_yaw;
    wrench_msg->wrench.torque.y = torque_robot.x * sin_yaw + torque_robot.y * cos_yaw;
    wrench_msg->wrench.torque.z = torque_robot.z;
    
    wrench_msg->header.frame_id = "odom"; // ワールド座標系
  } else {
    // 【ロボット座標モード】
    wrench_msg->wrench.force = force_robot;
    wrench_msg->wrench.torque = torque_robot;
    wrench_msg->header.frame_id = "base_link"; // ロボット基準座標系
  }
  // --- ▲▲▲ ここまで分岐 ▲▲▲ ---
>>>>>>> Stashed changes

  pub_->publish(std::move(wrench_msg));
}

} // namespace joy2wrench

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<joy2wrench::Joy2WrenchStamped>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}