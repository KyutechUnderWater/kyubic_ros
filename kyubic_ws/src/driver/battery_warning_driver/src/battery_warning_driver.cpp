/**
 * @file battery_warning_driver.cpp
 * @brief Battery Warning driver implementation
 */

#include "battery_warning_driver/battery_warning_driver.hpp"

using namespace std::chrono_literals;

namespace battery_warning_driver
{

BatteryWarningDriver::BatteryWarningDriver() : Node("battery_warning_driver")
{
  // パラメータ (ポート番号は他と被らないように適宜変更してください)
  sub_port = this->declare_parameter("sub_port", 9003);
  timeout_ms = this->declare_parameter("timeout", 1000);  // 1秒タイムアウト

  // タイムアウト管理オブジェクトの初期化
  timeout_ = std::make_shared<timer::Timeout>(this->get_clock()->now(), timeout_ms);

  // ROSパブリッシャの作成
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = create_publisher<RosBattery>("battery_warning", qos);

  // Protolinkサブスクライバの初期化
  protolink_subscriber_ = std::make_shared<protolink::udp_protocol::Subscriber<ProtoBattery>>(
    io_context_, sub_port, [this](const auto & _msg) {
      std::lock_guard<std::mutex> lock(mutex_);

      // データを受信したのでタイムアウトをリセット
      timeout_->reset(this->get_clock()->now());

      // 変換 (Proto -> ROS)
      RosBattery msg = convert(_msg);

      // ヘッダーとステータスの付与
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "battery_link";
      msg.status.id = common_msgs::msg::Status::NORMAL;

      // パブリッシュ
      pub_->publish(msg);

      // ログ出力 (bool値を文字列で表示)
      RCLCPP_INFO(
        this->get_logger(), "Subscribe & Publish ---> Leak: %s, RTC: %s",
        msg.battery_leak ? "TRUE" : "false", msg.battery_rtc ? "TRUE" : "false");
    });

  RCLCPP_INFO(this->get_logger(), "BatteryWarning Driver Listening on Port: %d", sub_port);

  // タイムアウト監視ループ (100msごと)
  timer_ = create_wall_timer(100ms, std::bind(&BatteryWarningDriver::_check_timeout, this));
}

void BatteryWarningDriver::_check_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (timeout_ms == 0) return;

  // タイムアウトした場合
  if (timeout_->check(this->get_clock()->now())) {
    auto msg = std::make_unique<RosBattery>();

    msg->header.stamp = this->get_clock()->now();
    msg->status.id = common_msgs::msg::Status::ERROR;  // ステータスをエラーに設定

    pub_->publish(std::move(msg));

    RCLCPP_ERROR(
      this->get_logger(),
      //   *this->get_clock(),
      //   1000, // 1秒に1回警告ログを出す
      "BatteryWarning driver timeout: %lu [ns]", timeout_->get_elapsed_time());
  }
}

}  // namespace battery_warning_driver

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<battery_warning_driver::BatteryWarningDriver>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}