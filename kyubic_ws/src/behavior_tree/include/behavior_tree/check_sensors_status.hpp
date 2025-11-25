/**
 * @file check_sensors_status.hpp
 * @brief Check sensors status
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details 各種センサ情報を取得し，エラーが出ているか確認する
 ************************************************************/

#ifndef _CHECK_SENSORS_STATUS_HPP
#define _CHECK_SENSORS_STATUS_HPP

#include <behaviortree_cpp/condition_node.h>

#include <common_msgs/msg/status.hpp>
#include <driver_msgs/msg/bool_stamped.hpp>
#include <driver_msgs/msg/depth.hpp>
#include <driver_msgs/msg/dvl.hpp>
#include <driver_msgs/msg/imu.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <timer/timeout.hpp>

/**
 * @brief Condition node to check the health status of robot sensors.
 */
class CheckSensorsStatus : public BT::ConditionNode
{
public:
  /**
   * @brief Constructor for the CheckSensorsStatus node.
   * @param name The name of the node in the behavior tree.
   * @param config The configuration of the node.
   * @param ros_node Shared pointer to the ROS 2 node used for subscriptions.
   */
  CheckSensorsStatus(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub,
    rclcpp::Node::SharedPtr ros_node);

  /**
   * @brief Defines the input and output ports for this node.
   * @return A list of ports.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Checks the current status of the sensors.
   * @return BT::NodeStatus::SUCCESS if sensors are healthy.
   *         BT::NodeStatus::FAILURE if any sensor reports an error (status == 2).
   */
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub_;
  rclcpp::Subscription<driver_msgs::msg::IMU>::SharedPtr imu_sub_;
  rclcpp::Subscription<driver_msgs::msg::Depth>::SharedPtr depth_sub_;
  rclcpp::Subscription<driver_msgs::msg::DVL>::SharedPtr dvl_sub_;
  rclcpp::Subscription<driver_msgs::msg::BoolStamped>::SharedPtr leak_sub_;

  void logger(rclcpp::Time now);

  /**
   * @brief センサーごとの監視データを管理する構造体
   */
  struct SensorMonitor
  {
    std::mutex mutex;
    std::shared_ptr<timer::Timeout> timer;
    uint8_t status;

    SensorMonitor(rclcpp::Time now, int64_t timeout_ns)
    : status(common_msgs::msg::Status::ERROR)  // Default is error
    {
      timer = std::make_shared<timer::Timeout>(now, timeout_ns);
    }

    /**
     * @brief If status is not Error, time reset
     */
    void update(uint8_t status, rclcpp::Time now)
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (status != common_msgs::msg::Status::ERROR) {
        timer->reset(now);
      }
    }

    /**
     * @brief If it has timed out, false
     */
    bool is_healthy(rclcpp::Time now)
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (timer->check(now)) {
        return false;
      }
      return true;
    }
  };

  /**
   * @brief Callback func generator
   * @tparam MsgT ROS2 message type
   * @tparam ExtraOps function of extra process
   */
  template <typename MsgT, typename ExtraOps>
  auto create_cb(std::shared_ptr<SensorMonitor> monitor_ptr, ExtraOps extra_process)
  {
    return [this, monitor_ptr, extra_process](const MsgT::ConstSharedPtr & msg) {
      monitor_ptr->update(msg->status.id, this->ros_node_->get_clock()->now());
      extra_process(msg);
    };
  }

  std::shared_ptr<SensorMonitor> imu_monitor_;
  std::shared_ptr<SensorMonitor> depth_monitor_;
  std::shared_ptr<SensorMonitor> dvl_monitor_;
  std::shared_ptr<SensorMonitor> leak_monitor_;
};

#endif  // _CHECK_SENSORS_STATUS_HPP
