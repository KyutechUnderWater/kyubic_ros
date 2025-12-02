
#include <chrono>
#include <iomanip>  // std::setprecision用
#include <iostream>

#include "behaviortree_cpp/loggers/abstract_logger.h"

class FilteredStdCoutLogger : public BT::StatusChangeLogger
{
public:
  FilteredStdCoutLogger(const BT::Tree & tree) : BT::StatusChangeLogger(tree.rootNode()) {}

  void callback(
    BT::Duration timestamp, const BT::TreeNode & node, BT::NodeStatus prev_status,
    BT::NodeStatus status) override
  {
    // 1. 除外フィルタ（UpdateMode, CheckSensorStatus を無視）
    if (
      (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::RUNNING ||
       status == BT::NodeStatus::IDLE) &&
      (node.name() == "UpdateMode" || node.name() == "CheckSensorsStatus" ||
       node.name() == "RetryUntilSuccessful")) {
      return;
    }

    // 2. 時刻の計算 (マイクロ秒 -> 秒)
    double time_sec = std::chrono::duration<double>(timestamp).count();

    // 3. 色の選択
    const char * color_code = resetColor();
    switch (status) {
      case BT::NodeStatus::SUCCESS:
        color_code = greenColor();
        break;
      case BT::NodeStatus::FAILURE:
        color_code = redColor();
        break;
      case BT::NodeStatus::RUNNING:
        color_code = yellowColor();
        break;
      case BT::NodeStatus::IDLE:
        color_code = resetColor();
        break;
      case BT::NodeStatus::SKIPPED:
        color_code = cyanColor();
        break;
    }

    // 4. フォーマット出力
    // [Time] [NodeName]: PREV -> NEW (Colorized)
    std::cout << "[" << std::fixed << std::setprecision(3) << time_sec << "s] " << "["
              << node.name() << "]: " << toStr(prev_status) << " -> " << color_code << toStr(status)
              << resetColor() << std::endl;
  }

  void flush() override { std::cout << std::flush; }

private:
  // ANSIエスケープシーケンス定義
  const char * redColor() { return "\033[31m"; }
  const char * greenColor() { return "\033[32m"; }
  const char * yellowColor() { return "\033[33m"; }
  const char * cyanColor() { return "\033[36m"; }
  const char * resetColor() { return "\033[0m"; }
};
