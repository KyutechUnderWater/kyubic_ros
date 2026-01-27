/**
 * @file lifecycle_stats_check_base.hpp
 * @brief Template class for checking the state of a Lifecycle Node
 * @author R.Ohnishi
 * @date 2026/01/21
 *
 * @details Lifecycle Nodeの状態をチェックする
 ********************************************************************/

#ifndef _LIFECYCLE_STATUS_CHECK_BASE_HPP
#define _LIFECYCLE_STATUS_CHECK_BASE_HPP

#include <rmw/qos_profiles.h>

#include <chrono>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <system_health_check/base_class/system_check_base.hpp>

namespace system_health_check::base
{

/**
 * @brief Base class for checking the state of a Lifecycle Node.
 * Connects to the `{node_name}/get_state` service to verify the current state.
 */
class LifecycleStatusCheckBase : public SystemCheckBase
{
protected:
  /**
   * @brief Configuration for the check.
   * @param target_node_name Name of the lifecycle node (e.g., "/camera_driver")
   * @param expected_state_id ID of the expected state (e.g., lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
   * @param timeout_ms Timeout in milliseconds
   */
  void set_config(
    const std::string target_node_name, const std::string expected_state, const uint32_t timeout_ms)
  {
    target_node_name_ = target_node_name;
    expected_state_id_ = parse_state_string(expected_state);
    timeout_ms_ = timeout_ms;
  };

  virtual void prepare_check(rclcpp::Node::SharedPtr node) override = 0;

private:
  std::string target_node_name_;
  uint8_t expected_state_id_ = 0;
  uint32_t timeout_ms_ = 0;
  std::string status_msg_;

  std::string get_unique_id() const override
  {
    return "LifecycleStatusCheckBase::" + target_node_name_ +
           "::" + std::to_string(expected_state_id_);
  }

  bool check_impl(rclcpp::Node::SharedPtr node) override
  {
    if (target_node_name_.empty() || timeout_ms_ == 0) {
      status_msg_ = "Not set target_node_name or timeout. Please invoke `set_config()` function.";
      return false;
    }

    // 1. サービス名の構築 (標準: /node_name/get_state)
    // 名前空間がある場合は調整が必要ですが、基本的にはこれで動作します
    std::string service_name = target_node_name_ + "/get_state";

    auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // 2. クライアントの作成
    auto client = node->create_client<lifecycle_msgs::srv::GetState>(
      service_name, rclcpp::ServicesQoS(), callback_group);

    // 3. サービスの存在確認 (Wait for service)
    if (!client->wait_for_service(std::chrono::milliseconds(timeout_ms_ / 2))) {
      status_msg_ = "Service not found: " + service_name;
      return false;  // FAIL (Service not available)
    }

    // 4. リクエストの送信
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = client->async_send_request(request);

    // 5. レスポンス待機 (Wait for response)
    std::future_status status = future.wait_for(std::chrono::milliseconds(timeout_ms_ / 2));

    if (status == std::future_status::ready) {
      try {
        auto response = future.get();
        uint8_t current_state_id = response->current_state.id;
        std::string current_state_label = response->current_state.label;

        // 6. 状態の検証 (Validate)
        if (validate_state(current_state_id)) {
          status_msg_ = "State valid: " + current_state_label +
                        " (ID: " + std::to_string(current_state_id) + ")";
          return true;  // PASS
        } else {
          status_msg_ = "State mismatch. Expected ID: " + std::to_string(expected_state_id_) +
                        ", Actual: " + current_state_label + " (" +
                        std::to_string(current_state_id) + ")";
          return false;  // FAIL (Wrong state)
        }
      } catch (const std::exception & e) {
        status_msg_ = "Exception during service call: " + std::string(e.what());
        return false;
      }
    } else {
      status_msg_ = "Service call timeout";
      return false;  // FAIL (Timeout)
    }
  }

  std::string report_impl([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    return "Node: " + target_node_name_ + " | " + status_msg_;
  }

  /**
   * @brief Validates the current state based on the expected state ID.
   */
  bool validate_state(uint8_t current_id) { return current_id == expected_state_id_; }

  // string to id
  int parse_state_string(const std::string & state_str)
  {
    if (state_str == "unconfigured")
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;                           // 1
    if (state_str == "inactive") return lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;    // 2
    if (state_str == "active") return lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;        // 3
    if (state_str == "finalized") return lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;  // 4
    return 3;  // Default is `active`.
  }
};

}  // namespace system_health_check::base

#endif
