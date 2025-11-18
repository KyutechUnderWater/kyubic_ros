/**
 * @file check_sensors_state.hpp
 * @brief Check sensors state
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details 各種センサ情報を取得し，エラーが出ているか確認する
 ************************************************************/

#ifndef _CHECK_SENSORS_STATE_HPP
#define _CHECK_SENSORS_STATE_HPP

#include <behaviortree_cpp/condition_node.h>

#include <driver_msgs/msg/depth.hpp>
#include <driver_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Condition node to check the health status of robot sensors.
 */
class CheckSensorsState : public BT::ConditionNode
{
public:
  /**
   * @brief Constructor for the CheckSensorsState node.
   * @param name The name of the node in the behavior tree.
   * @param config The configuration of the node.
   * @param ros_node Shared pointer to the ROS 2 node used for subscriptions.
   */
  CheckSensorsState(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node);

  /**
   * @brief Defines the input and output ports for this node.
   * @return A list of ports.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Checks the current state of the sensors.
   * @return BT::NodeStatus::SUCCESS if sensors are healthy.
   *         BT::NodeStatus::FAILURE if any sensor reports an error (status == 2).
   */
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<driver_msgs::msg::IMU>::SharedPtr imu_sub_;
  rclcpp::Subscription<driver_msgs::msg::Depth>::SharedPtr depth_sub_;

  std::atomic<uint8_t> imu_state_;
  std::atomic<uint8_t> depth_state_;
};

#endif  // _CHECK_SENSORS_STATE_HPP
