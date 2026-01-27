/**
 * @file action_server_check_base.hpp
 * @brief Template class for checking Action Server/Client existence
 * @author R.Ohnishi
 * @date 2026/01/21
 *
 * @details Action Server/Clientの存在をチェックする
 ********************************************************************/

#ifndef _ACTION_SERVER_CHECK_BASE_HPP
#define _ACTION_SERVER_CHECK_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <system_health_check/base_class/system_check_base.hpp>

// RCL Action Graph headers
#include <rcl/allocator.h>
#include <rcl/types.h>
#include <rcl_action/graph.h>

#include <string>
#include <thread>

namespace system_health_check::base
{

/**
 * @brief Abstract base class for checking Action Server/Client existence.
 * Handles node iteration, name parsing, and self-exclusion.
 */
class ActionGraphCheckBase : public SystemCheckBase
{
protected:
  std::string action_name_;
  uint32_t timeout_ms_ = 1000;
  size_t expected_count_ = 1;

  void set_config(
    const std::string action_name, const uint32_t timeout_ms, size_t expected_count = 1)
  {
    action_name_ = action_name;
    timeout_ms_ = timeout_ms;
    expected_count_ = expected_count;
  };

  virtual void prepare_check(rclcpp::Node::SharedPtr node) override = 0;

  virtual bool check_entity_in_node(
    rcl_node_t * rcl_node, rcl_allocator_t * allocator, const std::string & node_name,
    const std::string & node_ns) = 0;

  virtual std::string get_target_type() const = 0;

private:
  std::string status_msg_;

  bool check_impl(rclcpp::Node::SharedPtr node) override
  {
    if (action_name_.empty()) {
      status_msg_ = "Action name not set. Please invoke `set_config()` function.";
      return false;
    }

    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::milliseconds(timeout_ms_);

    // RCLノードハンドルとアロケータの取得
    rcl_node_t * rcl_node = node->get_node_base_interface()->get_rcl_node_handle();
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // 自身の完全修飾名を取得 (除外用)
    std::string current_node_name = node->get_fully_qualified_name();

    while (true) {
      size_t count = 0;

      // 1. 全ノード名を取得
      auto node_names = node->get_node_names();

      // 2. 各ノードを巡回
      for (const auto & full_node_name : node_names) {
        // 自身の除外
        if (full_node_name == current_node_name) {
          continue;
        }

        // --- 名前空間とノード名の分離処理 ---
        std::string target_node_name;
        std::string target_node_ns;

        size_t last_slash = full_node_name.find_last_of('/');
        if (last_slash == std::string::npos || last_slash == 0) {
          target_node_ns = "/";
          target_node_name = (last_slash == 0) ? full_node_name.substr(1) : full_node_name;
        } else {
          target_node_ns = full_node_name.substr(0, last_slash);
          target_node_name = full_node_name.substr(last_slash + 1);
        }

        // --- 派生クラスごとのチェック実行 ---
        if (check_entity_in_node(rcl_node, &allocator, target_node_name, target_node_ns)) {
          count++;
        }
      }

      // 判定
      if (count >= expected_count_) {
        status_msg_ = "Found " + std::to_string(count) + " Action " + get_target_type() + "(s).";
        return true;  // PASS
      }

      // タイムアウト判定
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > timeout_duration) {
        status_msg_ = "Timeout. Found " + std::to_string(count) + " Action " + get_target_type() +
                      "(s) (Expected >= " + std::to_string(expected_count_) + ").";
        return false;  // FAIL
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  std::string report_impl([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    return "Action " + get_target_type() + ": " + action_name_ + " | " + status_msg_;
  }
};

/**
 * @brief Checks specifically for Action SERVERS.
 */
class ActionServerCheckBase : public ActionGraphCheckBase
{
private:
  std::string get_unique_id() const override
  {
    return "ActionServerCheckBase::" + action_name_ + "::" + std::to_string(expected_count_);
  }

  std::string get_target_type() const override { return "Server"; }

  bool check_entity_in_node(
    rcl_node_t * rcl_node, rcl_allocator_t * allocator, const std::string & node_name,
    const std::string & node_ns) override
  {
    rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
    bool found = false;

    // Server用のAPIを使用
    rcl_ret_t ret = rcl_action_get_server_names_and_types_by_node(
      rcl_node, allocator, node_name.c_str(), node_ns.c_str(), &names_and_types);

    if (ret == RCL_RET_OK) {
      for (size_t i = 0; i < names_and_types.names.size; ++i) {
        if (std::string(names_and_types.names.data[i]) == action_name_) {
          found = true;
          break;
        }
      }
    }

    rcl_ret_t fini_ret = rcl_names_and_types_fini(&names_and_types);
    (void)fini_ret;

    return found;
  }
};

/**
 * @brief Checks specifically for Action CLIENTS.
 */
class ActionClientCheckBase : public ActionGraphCheckBase
{
private:
  std::string get_unique_id() const override
  {
    return "ActionClientCheckBase::" + action_name_ + "::" + std::to_string(expected_count_);
  }

  std::string get_target_type() const override { return "Client"; }

  bool check_entity_in_node(
    rcl_node_t * rcl_node, rcl_allocator_t * allocator, const std::string & node_name,
    const std::string & node_ns) override
  {
    rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
    bool found = false;

    // Client用のAPIを使用
    rcl_ret_t ret = rcl_action_get_client_names_and_types_by_node(
      rcl_node, allocator, node_name.c_str(), node_ns.c_str(), &names_and_types);

    if (ret == RCL_RET_OK) {
      for (size_t i = 0; i < names_and_types.names.size; ++i) {
        if (std::string(names_and_types.names.data[i]) == action_name_) {
          found = true;
          break;
        }
      }
    }

    rcl_ret_t fini_ret = rcl_names_and_types_fini(&names_and_types);
    (void)fini_ret;

    return found;
  }
};

}  // namespace system_health_check::base

#endif
