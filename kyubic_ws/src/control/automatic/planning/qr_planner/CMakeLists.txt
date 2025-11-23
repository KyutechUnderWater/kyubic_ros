cmake_minimum_required(VERSION 3.8)
project(qr_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 依存関係の検索 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)

# メッセージパッケージ
find_package(planner_msgs REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(action_msgs REQUIRED)
# ★★★ ここに追加！ ★★★
find_package(driver_msgs REQUIRED)
find_package(common_msgs REQUIRED) # 多分これも要るだろう

# インクルードディレクトリ
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# --- コンポーネント作成 ---
add_library(qr_planner_component SHARED src/qr_planner.cpp)

target_include_directories(qr_planner_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- リンク設定 ---
ament_target_dependencies(qr_planner_component
  rclcpp
  rclcpp_action
  rclcpp_components
  action_msgs
  planner_msgs
  localization_msgs
  # ★★★ ここに追加！ ★★★
  driver_msgs
  common_msgs
)

target_link_libraries(qr_planner_component Eigen3::Eigen)

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

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --- エクスポート設定 ---
ament_export_include_directories(include)
ament_export_libraries(qr_planner_component)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_components
  planner_msgs
  localization_msgs
  driver_msgs # これは既にあって偉い
  common_msgs
)

ament_package()