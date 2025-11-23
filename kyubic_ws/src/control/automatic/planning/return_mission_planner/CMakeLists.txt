cmake_minimum_required(VERSION 3.8)
project(return_mission_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必要なパッケージをfind_packageする
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(planner_msgs REQUIRED)

include_directories(include)

# 自動的に依存関係を解決してビルド設定を行う
ament_auto_find_build_dependencies() 

ament_auto_add_library(return_planner_component SHARED
  src/return_planner.cpp
)

rclcpp_components_register_node(return_planner_component
  PLUGIN "planner::ReturnPlanner"
  EXECUTABLE return_planner_node
)

install(TARGETS
  return_planner_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ヘッダーファイルのインストール
install(DIRECTORY include/
  DESTINATION include
)

# Launchファイルのインストール
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()