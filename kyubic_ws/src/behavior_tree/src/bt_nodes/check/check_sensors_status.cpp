/**
 * @file check_sensors_status.cpp
 * @brief Check sensors status
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details 各種センサ情報を取得し，エラーが出ているか確認する
 ************************************************************/

#include "behavior_tree/check/check_sensors_status.hpp"

CheckSensorsStatus::CheckSensorsStatus(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub, rclcpp::Node::SharedPtr ros_node)
: BT::ConditionNode(name, config), ros_node_(ros_node), logger_pub_(logger_pub)
{
  imu_monitor_ = std::make_shared<SensorMonitor>(ros_node_->get_clock()->now(), 2 * 1e9);
  depth_monitor_ = std::make_shared<SensorMonitor>(ros_node_->get_clock()->now(), 2 * 1e9);
  dvl_monitor_ = std::make_shared<SensorMonitor>(ros_node_->get_clock()->now(), 2 * 1e9);
  leak_monitor_ = std::make_shared<SensorMonitor>(ros_node_->get_clock()->now(), 2 * 1e9);

  auto no_op = [](const auto &) {};

  auto leak_process = [this](const driver_msgs::msg::BoolStamped::ConstSharedPtr & _msg) {
    this->setOutput("leak", _msg->data);
  };

  imu_sub_ = ros_node_->create_subscription<driver_msgs::msg::IMU>(
    "/driver/imu", 1, create_cb<driver_msgs::msg::IMU>(imu_monitor_, no_op));
  depth_sub_ = ros_node_->create_subscription<driver_msgs::msg::Depth>(
    "/driver/depth", 1, create_cb<driver_msgs::msg::Depth>(depth_monitor_, no_op));
  dvl_sub_ = ros_node_->create_subscription<driver_msgs::msg::DVL>(
    "/driver/dvl", 1, create_cb<driver_msgs::msg::DVL>(dvl_monitor_, no_op));
  leak_sub_ = ros_node_->create_subscription<driver_msgs::msg::BoolStamped>(
    "/driver/leak", 1, create_cb<driver_msgs::msg::BoolStamped>(leak_monitor_, leak_process));
}

BT::PortsList CheckSensorsStatus::providedPorts()
{
  return {BT::OutputPort("leak", "current leak sensor value")};
}

void CheckSensorsStatus::logger(rclcpp::Time now)
{
  std::string s = std::format(
    "imu: {}, depth: {}, dvl: {}, leak: {}", imu_monitor_->is_healthy(now),
    depth_monitor_->is_healthy(now), dvl_monitor_->is_healthy(now), leak_monitor_->is_healthy(now));

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "[CheckSensorsStatus] " + s;
  logger_pub_->publish(std::move(msg));
}

BT::NodeStatus CheckSensorsStatus::tick()
{
  auto now = ros_node_->get_clock()->now();

  logger(now);

  if (
    imu_monitor_->is_healthy(now) || depth_monitor_->is_healthy(now) ||
    dvl_monitor_->is_healthy(now) || leak_monitor_->is_healthy(now)) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}
