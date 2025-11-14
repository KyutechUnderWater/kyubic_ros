cmake_minimum_required(VERSION 3.8)
project(robot_bt_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# ... (中略) ...
find_package(custom_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# [修正] PDLAパッケージを検索
find_package(projection_dynamic_look_ahead_planner REQUIRED)
find_package(wrench_planner REQUIRED)


# ... (manual_node, emergency_node のビルド設定) ...

# [修正] WaypointNode
add_executable(waypoint_node src/lifecycle_nodes/waypoint_node.cpp)
ament_target_dependencies(waypoint_node 
  rclcpp 
  rclcpp_lifecycle
  rclcpp_action
  # [修正] PDLAのAction定義への依存を追加
  projection_dynamic_look_ahead_planner
)
rclcpp_components_register_node(waypoint_node PLUGIN robot_controller::WaypointNode)

# ... (bt_manager_node のビルド設定) ...

# [修正] bt_manager_node にも PDLA Action の依存関係が必要
# (もし BT ノードが Action Client を使う場合。今回は不要だが念のため)
ament_target_dependencies(bt_manager_node
  rclcpp
  # ... (中略) ...
  lifecycle_msgs
  # [修正] 
  projection_dynamic_look_ahead_planner
)

# ... (以降、変更なし) ...