cmake_minimum_required(VERSION 3.8)
project(qr_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Export compile command.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- 依存関係の検索 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# メッセージパッケージ
find_package(planner_msgs REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(driver_msgs REQUIRED)

include_directories(include)

# --- コンポーネント作成 ---
add_library(qr_planner_component SHARED src/qr_planner.cpp)

# --- リンク設定 ---
ament_target_dependencies(qr_planner_component
  rclcpp
  rclcpp_action
  rclcpp_components
  action_msgs
  planner_msgs
  localization_msgs
  driver_msgs
)

# ノード登録
rclcpp_components_register_node(qr_planner_component
  PLUGIN "planner::QRPlanner"
  EXECUTABLE qr_planner_node
)

# --- インストール設定 ---
install(TARGETS qr_planner_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
