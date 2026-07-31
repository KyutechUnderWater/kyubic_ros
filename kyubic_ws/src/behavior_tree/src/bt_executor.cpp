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
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "driver_msgs/msg/bool_stamped.hpp"

#include "behavior_tree/filtered_logger.hpp"

// Custom BT Node
#include "behavior_tree/always_running.hpp"
#include "behavior_tree/check/battery_check.hpp"
#include "behavior_tree/check/check_sensors_status.hpp"
#include "behavior_tree/find_pinger_action.hpp"
#include "behavior_tree/lifecycle_manager.hpp"
#include "behavior_tree/qr_action.hpp"
#include "behavior_tree/reset_localization.hpp"
#include "behavior_tree/talker.hpp"
#include "behavior_tree/update_mode.hpp"
#include "behavior_tree/waypoint_action.hpp"

// BlueROV固有のBT葉ノード(bluerov_control_bt_nodesパッケージ)
#include "bluerov_control_bt_nodes/capture_mission_origin.hpp"
#include "bluerov_control_bt_nodes/check_buoy_detected.hpp"
#include "bluerov_control_bt_nodes/check_pinger_found.hpp"
#include "bluerov_control_bt_nodes/check_ros_bool_param.hpp"
#include "bluerov_control_bt_nodes/go_to_blackboard_target.hpp"
#include "bluerov_control_bt_nodes/go_to_depth.hpp"
#include "bluerov_control_bt_nodes/load_mission_params.hpp"

using namespace behavior_tree;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 1. Create the Node
  auto node = std::make_shared<rclcpp::Node>("bt_executor_node");
  std::string bt_xml_path = node->declare_parameter<std::string>("bt_xml_file", "");
  // 空なら従来通りXML自身のmain_tree_to_execute属性を使う。非空ならXML内の指定した
  // BehaviorTree IDをrootとして実行する(状態ごとの単体テスト用。BlueROVの
  // bluerov_control_bt_nodesが導入した仕組みで、既存の使い方には影響しない)。
  std::string main_tree_id = node->declare_parameter<std::string>("main_tree_id", "");

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logger_pub =
    node->create_publisher<std_msgs::msg::String>("logger", 10);

  // 2. Initialize BT Factory
  BT::BehaviorTreeFactory factory;

  // 3. Register nodes
  factory.registerNodeType<AlwaysRunning>("AlwaysRunning", logger_pub);
  factory.registerNodeType<CheckSensorsStatus>("CheckSensorsStatus", logger_pub, node);
  factory.registerNodeType<BatteryCheck>("BatteryCheck", logger_pub, node);
  factory.registerNodeType<UpdateMode>("UpdateMode", logger_pub, node, 0);
  factory.registerNodeType<LifecycleManager>("LifecycleManager", logger_pub, node);
  factory.registerNodeType<WaypointAction>("WaypointAction", logger_pub, node);
  factory.registerNodeType<QrAction>("QrAction", logger_pub, node);
  factory.registerNodeType<ResetLocalization>("ResetLocalization", logger_pub, node);
  factory.registerNodeType<FindPingerAction>("FindPingerAction", logger_pub, node);
  factory.registerNodeType<Talker>("Talker", node);
  factory.registerNodeType<bluerov_control_bt_nodes::GoToDepth>("GoToDepth", node);
  factory.registerNodeType<bluerov_control_bt_nodes::GoToBlackboardTarget>(
    "GoToBlackboardTarget", node);
  factory.registerNodeType<bluerov_control_bt_nodes::CheckPingerFound>("CheckPingerFound", node);
  factory.registerNodeType<bluerov_control_bt_nodes::CheckBuoyDetected>("CheckBuoyDetected", node);
  factory.registerNodeType<bluerov_control_bt_nodes::CheckRosBoolParam>("CheckRosBoolParam", node);
  factory.registerNodeType<bluerov_control_bt_nodes::LoadMissionParams>("LoadMissionParams", node);
  factory.registerNodeType<bluerov_control_bt_nodes::CaptureMissionOrigin>(
    "CaptureMissionOrigin", node);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("mode", "manual");

  // リードスイッチトリガー待機 (wait_for_trigger=true のとき BT 開始前に待つ)
  bool wait_for_trigger = node->declare_parameter<bool>("wait_for_trigger", false);
  if (wait_for_trigger) {
    RCLCPP_INFO(node->get_logger(), "Waiting for reed switch trigger on /driver/blue_rov/mission_start_trigger ...");
    bool triggered = false;
    auto trigger_sub = node->create_subscription<driver_msgs::msg::BoolStamped>(
      "/driver/blue_rov/mission_start_trigger", 10,
      [&triggered](driver_msgs::msg::BoolStamped::SharedPtr msg) {
        if (msg->data) triggered = true;
      });
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !triggered) {
      rclcpp::spin_some(node);
      rate.sleep();
    }
    RCLCPP_INFO(node->get_logger(), "Reed switch triggered! Starting behavior tree.");
  }

  // 4. Retrieve path to BT XML file
  std::string xml_path = bt_xml_path;
  if (xml_path[0] != '/') {
    std::string pkg_share;
    try {
      pkg_share = ament_index_cpp::get_package_share_directory("behavior_tree");
      xml_path = pkg_share + "/bt_xml/" + bt_xml_path;
      RCLCPP_INFO(node->get_logger(), "Resolved relative path");
    } catch (const ament_index_cpp::PackageNotFoundError & e) {
      RCLCPP_ERROR(node->get_logger(), "Package 'behavior_tree' not found. %s", e.what());
      rclcpp::shutdown();
      return 1;
    }
  }
  RCLCPP_INFO(node->get_logger(), "Loading BT XML: %s", xml_path.c_str());

  // 5. Create tree
  BT::Tree tree;
  try {
    if (main_tree_id.empty()) {
      tree = factory.createTreeFromFile(xml_path, blackboard);
    } else {
      RCLCPP_INFO(node->get_logger(), "main_tree_id指定あり: '%s' をrootとして実行します",
        main_tree_id.c_str());
      factory.registerBehaviorTreeFromFile(xml_path);
      tree = factory.createTree(main_tree_id, blackboard);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create tree: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Print Tree structure
  printTreeRecursively(tree.rootNode());

  // Standard output logger
  // auto logger = std::make_unique<BT::StdCoutLogger>(tree);
  FilteredStdCoutLogger logger_filtered(tree);

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
