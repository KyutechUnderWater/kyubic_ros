#pragma once
#include <rclcpp/rclcpp.hpp>
#include <system_check/base_class/system_check_base.hpp>

// RCL (C API) のアクション用ヘッダー
#include <rcl/allocator.h>
#include <rcl/types.h>
#include <rcl_action/graph.h>

#include <string>
#include <thread>

namespace system_check
{

/**
 * @brief Checks if a specific Action Server is available using low-level RCL API.
 */
class ActionServerCheckBase : public system_check::SystemCheckBase
{
public:
  virtual bool check(rclcpp::Node::SharedPtr node) override
  {
    if (action_name_.empty()) {
      status_msg_ = "Action name not set. Please invoke `set_config()` function.";
      return false;
    }

    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::milliseconds(timeout_ms_);

    // C API用のノードハンドルとアロケータを取得
    rcl_node_t * rcl_node = node->get_node_base_interface()->get_rcl_node_handle();
    rcl_allocator_t allocator = rcl_get_default_allocator();

    while (true) {
      // 1. 結果格納用の構造体をゼロ初期化
      rcl_names_and_types_t action_names_and_types = rcl_get_zero_initialized_names_and_types();

      // 2. アクション一覧の取得 (低レイヤAPI)
      rcl_ret_t ret = rcl_action_get_names_and_types(rcl_node, &allocator, &action_names_and_types);

      bool found = false;
      if (ret == RCL_RET_OK) {
        // 3. 取得したリスト内を検索
        for (size_t i = 0; i < action_names_and_types.names.size; ++i) {
          // C文字列をstd::stringとして比較
          if (std::string(action_names_and_types.names.data[i]) == action_name_) {
            found = true;
            break;
          }
        }
      } else {
        // 取得失敗時のログ（必要に応じて）
        // RCLCPP_WARN(node->get_logger(), "rcl_action_get_names_and_types failed: %d", ret);
      }

      // 4. 重要: 確保されたメモリを必ず解放する
      rcl_ret_t fini_ret = rcl_names_and_types_fini(&action_names_and_types);
      (void)fini_ret;  // 未使用警告の抑制

      if (found) {
        status_msg_ = "Action server found.";
        return true;  // PASS
      }

      // Check timeout
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > timeout_duration) {
        status_msg_ = "Timeout. Action server not found.";
        return false;  // FAIL
      }

      // Sleep briefly
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  std::string report([[maybe_unused]] rclcpp::Node::SharedPtr node) override
  {
    return "Action: " + action_name_ + " | " + status_msg_;
  }

  void set_config(const std::string action_name, const uint32_t timeout_ms)
  {
    // 必要に応じてリマップ済みの名前や正規化された名前（"/"始まり）を扱うよう注意
    action_name_ = action_name;
    timeout_ms_ = timeout_ms;
  };

private:
  std::string action_name_;
  uint32_t timeout_ms_ = 1000;
  std::string status_msg_;
};

}  // namespace system_check
