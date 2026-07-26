// BlueROV ドライバー・ローカライゼーションのトピックを購読し、
// データが届いているかをログで確認する受信専用の診断ノード。
// パブリッシャーは持たないため、車両へコマンドは送信しない。
#include "blue_control/blue_control_listener.hpp"

namespace blue_control
{

namespace
{
// ログ出力の間隔 [ms]。高頻度トピックでもターミナルが埋まらないよう制限する。
constexpr int kThrottleMs = 2000;
}  // namespace

BlueControlListener::BlueControlListener(const rclcpp::NodeOptions & options)
: Node("blue_control_listener", options)
{
  // QoS: キュー深さ 10。標準的なセンサー購読向け設定。
  const auto qos = rclcpp::QoS(10);

  // --- 車両非依存の共通トピック（どのドライバースタックでも publish される） ---
  imu_sub_ = create_subscription<driver_msgs::msg::IMU>(
    "/driver/imu", qos, std::bind(&BlueControlListener::on_imu, this, std::placeholders::_1));
  depth_sub_ = create_subscription<driver_msgs::msg::Depth>(
    "/driver/depth", qos, std::bind(&BlueControlListener::on_depth, this, std::placeholders::_1));
  dvl_sub_ = create_subscription<driver_msgs::msg::DVL>(
    "/driver/dvl", qos, std::bind(&BlueControlListener::on_dvl, this, std::placeholders::_1));

  // --- BlueROV 固有トピック（mavlink_driver / dvl75_driver） ---
  dvl75_sub_ = create_subscription<blue_rov_msgs::msg::DVL75>(
    "/driver/blue_rov/dvl75_driver/dvl75", qos,
    std::bind(&BlueControlListener::on_dvl75, this, std::placeholders::_1));
  vehicle_state_sub_ = create_subscription<driver_msgs::msg::VehicleState>(
    "/driver/blue_rov/mavlink_driver/vehicle_state", qos,
    std::bind(&BlueControlListener::on_vehicle_state, this, std::placeholders::_1));
  power_state_sub_ = create_subscription<driver_msgs::msg::PowerState>(
    "/driver/blue_rov/mavlink_driver/power_state", qos,
    std::bind(&BlueControlListener::on_power_state, this, std::placeholders::_1));

  // --- センサー融合の出力（localization_components.launch.py の起動が必要） ---
  odom_sub_ = create_subscription<localization_msgs::msg::Odometry>(
    "/localization/odom", qos,
    std::bind(&BlueControlListener::on_odom, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "blue_control_listener started (subscribe-only, no publishers)");
}

// IMU 受信: 姿勢角（度）をログ出力
void BlueControlListener::on_imu(const driver_msgs::msg::IMU::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), kThrottleMs, "imu: orient(deg)=[%.2f, %.2f, %.2f]", msg->orient.x,
    msg->orient.y, msg->orient.z);
}

// 深度センサー受信: 水深 [m] と水温 [°C] をログ出力
void BlueControlListener::on_depth(const driver_msgs::msg::Depth::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), kThrottleMs, "depth: depth=%.3f m, temperature=%.2f C", msg->depth,
    msg->temperature);
}

// 共通 DVL 受信: 速度の有効性と海底高度 [m] をログ出力
void BlueControlListener::on_dvl(const driver_msgs::msg::DVL::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), kThrottleMs, "dvl: velocity_valid=%d altitude=%.3f m",
    msg->velocity_valid, msg->altitude);
}

// DVL-75 固有データ受信: 海底ロック状態と高度 [m] をログ出力
void BlueControlListener::on_dvl75(const blue_rov_msgs::msg::DVL75::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), kThrottleMs, "dvl75: bottom_lock=%d altitude=%.3f m",
    msg->bottom_lock, msg->altitude);
}

// 車両状態受信: MAVLink 接続状態とアーム（モーター有効）状態をログ出力
void BlueControlListener::on_vehicle_state(const driver_msgs::msg::VehicleState::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), kThrottleMs, "vehicle_state: connected=%d armed=%d", msg->connected,
    msg->armed);
}

// 電源状態受信: ログ用・実測の電圧 [V] をログ出力
void BlueControlListener::on_power_state(const driver_msgs::msg::PowerState::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), kThrottleMs, "power_state: log_voltage=%.2f V act_voltage=%.2f V",
    msg->log_voltage, msg->act_voltage);
}

// オドメトリ受信: 推定位置 (x, y) と深度 [m] をログ出力
void BlueControlListener::on_odom(const localization_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), kThrottleMs, "odom: position=[%.3f, %.3f] depth=%.3f m",
    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z_depth);
}

}  // namespace blue_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // ノードを生成し、コールバック待ちのイベントループを開始
  rclcpp::spin(std::make_shared<blue_control::BlueControlListener>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
