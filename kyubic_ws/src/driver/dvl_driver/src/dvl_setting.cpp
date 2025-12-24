/**
 * @file dvl_setting.cpp
 * @brief DVL Configuration Client Implementation
 */

#include "dvl_driver/dvl_setting.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <iostream>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace dvl_driver
{

// ヘルパー: 文字列内のすべての空白を削除 (スペース対策)
std::string remove_all_spaces(std::string str)
{
  str.erase(std::remove_if(str.begin(), str.end(), ::isspace), str.end());
  return str;
}

DVLSetting::DVLSetting() : Node("dvl_setting")
{
  std::string config_file_param = this->declare_parameter("config_file", "config/dvl_config.txt");
  fs::path file_path(config_file_param);

  if (file_path.is_absolute()) {
    config_file_path_ = config_file_param;
  } else {
    try {
      std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("dvl_driver");
      fs::path share_path(package_share_directory);
      fs::path full_path = share_path / file_path;
      config_file_path_ = full_path.string();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get package share directory: %s", e.what());
    }
  }

  client_ = this->create_client<driver_msgs::srv::Command>("command");
  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for 'dvl_command' service...");
  }
}

DVLSetting::~DVLSetting() {}

bool DVLSetting::configure()
{
  auto commands = load_config_file(config_file_path_);
  if (commands.empty()) {
    RCLCPP_WARN(this->get_logger(), "No commands found in %s", config_file_path_.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "--- Starting DVL Configuration ---");
  bool overall_success = true;

  for (const auto & cmd : commands) {
    if (!execute_command(cmd)) {
      overall_success = false;
    }
    rclcpp::sleep_for(100ms);
  }

  if (overall_success) {
    RCLCPP_INFO(this->get_logger(), "--- Configuration Completed Successfully ---");
  } else {
    RCLCPP_ERROR(this->get_logger(), "--- Configuration Finished with ERRORS ---");
  }
  return overall_success;
}

bool DVLSetting::execute_command(const std::string & cmd)
{
  // 1. コマンド送信
  RCLCPP_INFO(this->get_logger(), "Sending: %s", cmd.c_str());
  std::string response = call_service(cmd);

  if (response.find("ERR") != std::string::npos || response.find("Error") != std::string::npos) {
    RCLCPP_ERROR(this->get_logger(), "  -> DVL Error Response: %s", response.c_str());
    return false;
  }

  // 2. 特殊コマンドは検証スキップ
  if (cmd == "CS") {
    RCLCPP_INFO(this->get_logger(), "  -> Start Ping sent.");
    return true;
  }
  if (cmd == "CK" || cmd.find("CR") == 0 || cmd == "===") {
    return true;
  }

  // 3. 検証ロジック (Verify)
  std::string query = get_query_command(cmd);
  if (query.empty() || query == cmd + "?") {
    return true;
  }

  // クエリ送信 (例: "#BM?")
  std::string verify_resp = call_service(query);

  // --- 修正箇所: 空白を除去して比較 ---
  // 設定コマンド "#BM8" -> "#BM8"
  std::string normalized_cmd = remove_all_spaces(cmd);
  // レスポンス ">#BM 8" -> ">#BM8"
  std::string normalized_resp = remove_all_spaces(verify_resp);

  if (normalized_resp.find(normalized_cmd) != std::string::npos) {
    // 成功: レスポンス内の空白を除いた部分に、設定値が含まれていた
    RCLCPP_INFO(this->get_logger(), "  -> OK (Verified)");
    return true;
  } else {
    // 失敗
    RCLCPP_WARN(
      this->get_logger(), "  -> VERIFY FAILED! Set: '%s', Query: '%s' Got: '%s'", cmd.c_str(),
      query.c_str(),
      verify_resp.c_str());  // ログには生のレスポンスを出すと分かりやすい
    return false;
  }
}

std::string DVLSetting::call_service(const std::string & command)
{
  auto request = std::make_shared<driver_msgs::srv::Command::Request>();
  request->command = command;
  auto result_future = client_->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    return result_future.get()->output;
  } else {
    return "Service Error";
  }
}

std::vector<std::string> DVLSetting::load_config_file(const std::string & path)
{
  std::vector<std::string> lines;
  std::ifstream file(path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open config file: %s", path.c_str());
    return lines;
  }
  std::string line;
  while (std::getline(file, line)) {
    size_t comment_pos = line.find(';');
    if (comment_pos != std::string::npos) line = line.substr(0, comment_pos);
    line = trim(line);
    if (!line.empty()) lines.push_back(line);
  }
  return lines;
}

std::string DVLSetting::get_query_command(const std::string & cmd)
{
  std::string prefix;
  for (char c : cmd) {
    if (std::isdigit(c) || c == '-' || c == '+' || c == ':') break;
    prefix += c;
  }
  if (prefix.empty()) return "";
  return prefix + "?";
}

std::string DVLSetting::trim(const std::string & str)
{
  const auto str_begin = str.find_first_not_of(" \t\r\n");
  if (str_begin == std::string::npos) return "";
  const auto str_end = str.find_last_not_of(" \t\r\n");
  return str.substr(str_begin, str_end - str_begin + 1);
}

}  // namespace dvl_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dvl_driver::DVLSetting>();
  bool success = node->configure();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
