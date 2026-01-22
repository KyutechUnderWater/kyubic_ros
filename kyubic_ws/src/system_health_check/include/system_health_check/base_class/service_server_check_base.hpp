#pragma once
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <system_health_check/base_class/system_health_check_base.hpp>
#include <thread>
#include <vector>

// RCL headers for Client check
#include <rcl/allocator.h>
#include <rcl/graph.h>
#include <rcl/types.h>

namespace system_health_check
{

/**
 * @brief Abstract base class for checking Service Server/Client existence.
 * Handles node iteration, name parsing, and self-exclusion logic.
 */
class ServiceGraphCheckBase : public system_health_check::SystemCheckBase
{
public:
  virtual bool check(rclcpp::Node::SharedPtr node) override
  {
    if (service_name_.empty()) {
      status_msg_ = "Service name not set. Please invoke `set_config()` function.";
      return false;
    }

    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::milliseconds(timeout_ms_);

    // 自身の完全修飾名を取得 (例: "/my_ns/checker_node")
    // これを使って自身をカウントから除外する
    std::string current_node_name = node->get_fully_qualified_name();

    while (true) {
      size_t count = 0;

      // 1. 全ノード名を取得
      auto node_names = node->get_node_names();

      // 2. 各ノードを巡回
      for (const auto & full_node_name : node_names) {
        // --- 自身の除外処理 ---
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

        // --- 派生クラスごとのカウント処理 ---
        if (check_entity_in_node(node, target_node_name, target_node_ns)) {
          count++;
        }
      }

      // 判定
      if (count >= expected_count_) {
        status_msg_ = "Found " + std::to_string(count) + " " + get_target_type() + "(s).";
        return true;  // PASS
      }

      // タイムアウト判定
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > timeout_duration) {
        status_msg_ = "Timeout. Found " + std::to_string(count) + " " + get_target_type() +
                      "(s) (Expected >= " + std::to_string(expected_count_) + ").";
        return false;  // FAIL
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  std::string report([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    return "Service " + get_target_type() + ": " + service_name_ + " | " + status_msg_;
  }

  void set_config(
    const std::string service_name, const uint32_t timeout_ms, size_t expected_count = 1)
  {
    service_name_ = service_name;
    timeout_ms_ = timeout_ms;
    expected_count_ = expected_count;
  };

protected:
  // 派生クラスで実装: そのノードにターゲットが含まれているか確認する
  virtual bool check_entity_in_node(
    rclcpp::Node::SharedPtr node, const std::string & node_name, const std::string & node_ns) = 0;

  // ログ表示用のタイプ名 ("Server" or "Client")
  virtual std::string get_target_type() const = 0;

  std::string service_name_;
  uint32_t timeout_ms_ = 1000;
  size_t expected_count_ = 1;
  std::string status_msg_;
};

/**
 * @brief Checks for Service SERVER existence.
 */
class ServiceServerCheckBase : public ServiceGraphCheckBase
{
protected:
  std::string get_target_type() const override { return "Server"; }

  bool check_entity_in_node(
    rclcpp::Node::SharedPtr node, const std::string & node_name,
    const std::string & node_ns) override
  {
    try {
      // get_service_names_and_types_by_node は Server のみを返す
      auto services = node->get_service_names_and_types_by_node(node_name, node_ns);
      return services.find(service_name_) != services.end();
    } catch (const std::exception &) {
      return false;
    }
  }
};

/**
 * @brief Checks for Service CLIENT existence.
 */
class ServiceClientCheckBase : public ServiceGraphCheckBase
{
protected:
  std::string get_target_type() const override { return "Client"; }

  bool check_entity_in_node(
    rclcpp::Node::SharedPtr node, const std::string & node_name,
    const std::string & node_ns) override
  {
    rcl_node_t * rcl_node = node->get_node_base_interface()->get_rcl_node_handle();
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_names_and_types_t client_names_and_types = rcl_get_zero_initialized_names_and_types();

    bool found = false;
    rcl_ret_t ret = rcl_get_client_names_and_types_by_node(
      rcl_node, &allocator, node_name.c_str(), node_ns.c_str(), &client_names_and_types);

    if (ret == RCL_RET_OK) {
      for (size_t i = 0; i < client_names_and_types.names.size; ++i) {
        if (std::string(client_names_and_types.names.data[i]) == service_name_) {
          found = true;
          break;
        }
      }
    }

    // 必ずメモリ解放
    rcl_ret_t fini_ret = rcl_names_and_types_fini(&client_names_and_types);
    (void)fini_ret;

    return found;
  }
};

}  // namespace system_health_check
