#ifndef JOY2WRENCH__JOY2WRENCH_HPP_
#define JOY2WRENCH__JOY2WRENCH_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <mutex>
#include <string>
#include <vector>

namespace joy2wrench
{
class Joy2WrenchStamped : public rclcpp::Node
{
public:
  Joy2WrenchStamped();

private:
<<<<<<< Updated upstream
  std::string device_name;
  double force_x_scale, force_y_scale, force_z_scale;
  double torque_x_scale, torque_z_scale;
=======
  // コールバック関数
  void _joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void _odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
>>>>>>> Stashed changes

  // パラメータを保持するメンバ変数
  std::string device_name;
  double force_x_scale;
  double force_y_scale;
  double force_z_scale;
  double torque_x_scale;
  double torque_z_scale;
  
  // Publisher, Subscriber
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // ロボットの現在の方位角（ヨー角）と、それを保護するミューテックス
  double current_yaw_ = 0.0;
  std::mutex yaw_mutex_;

  // --- ▼▼▼ モード切替用の変数を追加 ▼▼▼ ---
  bool is_absolute_mode_ = false; // 現在のモード（false: ロボット基準, true: 絶対座標基準）
  bool prev_button_state_ = false; // ボタンの前のフレームでの状態
  // --- ▲▲▲ ここまで追加 ▲▲▲ ---
};
} // namespace joy2wrench

#endif // JOY2WRENCH__JOY2WRENCH_HPP_