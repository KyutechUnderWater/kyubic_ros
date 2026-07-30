#include "bluerov_control_bt_nodes/check_pinger_found.hpp"

namespace bluerov_control_bt_nodes
{

CheckPingerFound::CheckPingerFound(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr ros_node)
: BT::StatefulActionNode(name, config), ros_node_(ros_node)
{
  std::string pinger_topic;
  if (!getInput("pinger_direction_topic", pinger_topic)) {
    pinger_topic = "pinger_direction";
  }
  std::string wrench_plan_topic;
  if (!getInput("wrench_plan_topic", wrench_plan_topic)) {
    wrench_plan_topic = "zoh_wrench_plan";
  }

  pinger_sub_ = ros_node_->create_subscription<planner_msgs::msg::PingerDirection>(
    pinger_topic, 10, std::bind(&CheckPingerFound::pingerCallback, this, std::placeholders::_1));
  wrench_plan_sub_ = ros_node_->create_subscription<planner_msgs::msg::WrenchPlan>(
    wrench_plan_topic, 10,
    std::bind(&CheckPingerFound::wrenchPlanCallback, this, std::placeholders::_1));
}

BT::PortsList CheckPingerFound::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "pinger_direction_topic", "pinger_direction", "sbl_estimation_nodeが発行するトピック"),
    BT::InputPort<std::string>(
      "wrench_plan_topic", "zoh_wrench_plan", "sbl_controller_nodeが発行し続ける追従目標トピック"),
    BT::InputPort<double>("pitch_threshold_deg", 30.0, "これを超えたら十分近いとみなす"),
    BT::InputPort<double>(
      "score_threshold", 1.0e6,
      "PingerDirection.scoreの上限ゲート(誤差。小さいほど良い)。既定は常に許可"),
    BT::OutputPort<double>("target_x", "発見時点のwrench_plan目標x[m](ブラックボードへ出力)"),
    BT::OutputPort<double>("target_y", "発見時点のwrench_plan目標y[m](ブラックボードへ出力)"),
    BT::OutputPort<double>("target_z", "発見時点のwrench_plan目標z[m](ブラックボードへ出力)")};
}

void CheckPingerFound::pingerCallback(const planner_msgs::msg::PingerDirection::SharedPtr msg)
{
  latest_pinger_ = msg;
}

void CheckPingerFound::wrenchPlanCallback(const planner_msgs::msg::WrenchPlan::SharedPtr msg)
{
  latest_wrench_plan_ = msg;
}

BT::NodeStatus CheckPingerFound::onStart() { return BT::NodeStatus::RUNNING; }

BT::NodeStatus CheckPingerFound::onRunning()
{
  if (!latest_pinger_ || !latest_wrench_plan_) {
    // pinger_direction/wrench_planのどちらかをまだ一度も受信していない。安全側に倒し待機する。
    return BT::NodeStatus::RUNNING;
  }

  double pitch_threshold_deg = 30.0;
  double score_threshold = 1.0e6;
  getInput("pitch_threshold_deg", pitch_threshold_deg);
  getInput("score_threshold", score_threshold);

  // TODO(要ハードウェア仕様確認): 不等号の向きはPingerDirection.pitchの「正=対象が下方向」
  // という定義を仮定している。実機で符号を検証すること(旧flow.py handle_search_hydrophoneと同じ)。
  const bool usable = latest_pinger_->score <= score_threshold;
  if (!usable || latest_pinger_->pitch <= pitch_threshold_deg) {
    return BT::NodeStatus::RUNNING;
  }

  setOutput("target_x", latest_wrench_plan_->targets.x);
  setOutput("target_y", latest_wrench_plan_->targets.y);
  setOutput("target_z", latest_wrench_plan_->targets.z);
  RCLCPP_INFO(
    ros_node_->get_logger(),
    "CheckPingerFound: 発見(pitch=%.1f, score=%.3f) target=(%.2f, %.2f, %.2f)",
    latest_pinger_->pitch, latest_pinger_->score, latest_wrench_plan_->targets.x,
    latest_wrench_plan_->targets.y, latest_wrench_plan_->targets.z);
  return BT::NodeStatus::SUCCESS;
}

void CheckPingerFound::onHalted() {}

}  // namespace bluerov_control_bt_nodes
