/**
 * @file bt_executor.cpp
 * @brief behavior tree executor
 * @author R.Ohnishi
 * @date 2025/11/18
 *
 * @details BehaviorTreeの実行
 ******************************/

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/controls/switch_node.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

// Custom BT Node
#include "behavior_tree/always_running.hpp"
#include "behavior_tree/check_sensors_status.hpp"
#include "behavior_tree/find_pinger_action.hpp"
#include "behavior_tree/lifecycle_manager.hpp"
#include "behavior_tree/qr_action.hpp"
#include "behavior_tree/reset_localization.hpp"
#include "behavior_tree/update_mode.hpp"
#include "behavior_tree/waypoint_action.hpp"

using namespace behavior_tree;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 1. Create the Node
  auto node = std::make_shared<rclcpp::Node>("bt_executor_node");
  std::string bt_xml_file = node->declare_parameter<std::string>("bt_xml_file", "");

  // 2. Initialize BT Factory
  BT::BehaviorTreeFactory factory;

  // 3. Register nodes
  factory.registerNodeType<AlwaysRunning>("AlwaysRunning");
  factory.registerNodeType<CheckSensorsStatus>("CheckSensorsStatus", node);
  factory.registerNodeType<UpdateMode>("UpdateMode", node);
  factory.registerNodeType<LifecycleManager>("LifecycleManager", node);
  factory.registerNodeType<WaypointAction>("WaypointAction", node);
  factory.registerNodeType<QrAction>("QrAction", node);
  factory.registerNodeType<ResetLocalization>("ResetLocalization", node);
  factory.registerNodeType<FindPingerAction>("FindPingerAction", node);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("mode", "manual");

  // 4. Retrieve path to BT XML file
  std::string pkg_share;
  try {
    pkg_share = ament_index_cpp::get_package_share_directory("behavior_tree");
  } catch (const ament_index_cpp::PackageNotFoundError & e) {
    RCLCPP_ERROR(node->get_logger(), "Package 'behavior_tree' not found. %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  std::string xml_file = pkg_share + "/bt_xml/" + bt_xml_file;
  RCLCPP_INFO(node->get_logger(), "Loading BT XML: %s", xml_file.c_str());

  // 5. Create tree
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(xml_file, blackboard);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create tree: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Print Tree structure
  printTreeRecursively(tree.rootNode());

  // Standard output logger
  auto logger = std::make_unique<BT::StdCoutLogger>(tree);

  // 6. Timer to TICK the tree
  rclcpp::TimerBase::SharedPtr timer;
  timer = node->create_wall_timer(std::chrono::milliseconds(10), [&tree, node, &timer]() {
    const BT::NodeStatus status = tree.tickOnce();
    RCLCPP_DEBUG(node->get_logger(), "Tree ticked, status: %s", BT::toStr(status).c_str());

    if (status != BT::NodeStatus::RUNNING) {
      RCLCPP_INFO(
        node->get_logger(), "Behavior Tree finished with status: %s. Shutting down...",
        BT::toStr(status).c_str());

      timer->cancel();
      rclcpp::shutdown();
    }
  });

  RCLCPP_INFO(node->get_logger(), "BT manager (main) spinning...");
  rclcpp::spin(node);

  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Shutdown complete.");
  return 0;
}
