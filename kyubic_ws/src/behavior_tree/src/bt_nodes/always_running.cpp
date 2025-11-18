/**
 * @file always_running.cpp
 * @brief Behavior Tree Node that always return running
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details 常にBT::NodeStatus::RUNNINGを返すBehaviorTreeのノード
 ****************************************************************/

#include "behavior_tree/always_running.hpp"

namespace behavior_tree
{

AlwaysRunning::AlwaysRunning(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList AlwaysRunning::providedPorts() { return {}; }

BT::NodeStatus AlwaysRunning::onStart() { return BT::NodeStatus::RUNNING; }

BT::NodeStatus AlwaysRunning::onRunning() { return BT::NodeStatus::RUNNING; }

void AlwaysRunning::onHalted() {}

}  // namespace behavior_tree
