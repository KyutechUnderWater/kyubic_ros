cmake_minimum_required(VERSION 3.8)
project(battery_warning_driver)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(driver_msgs REQUIRED)
find_package(protolink REQUIRED)
find_package(timer REQUIRED) 

# =========================================================
# Protolinkメッセージ生成
# =========================================================
add_protolink_message_from_ros_message("driver_msgs" "BatteryWarning")
# =========================================================

include_directories(include)

# =========================================================
# 【修正】ここが重要です！
# 先に add_executable を書いてターゲットを作成します
# =========================================================
add_executable(battery_warning_driver src/battery_warning_driver.cpp)

# その後に依存関係を追加します
ament_target_dependencies(battery_warning_driver
  rclcpp
  driver_msgs
  protolink
  timer
)

# =========================================================
# リンク設定 
# =========================================================
target_link_libraries(battery_warning_driver
  driver_msgs__BatteryWarning_proto
)
# =========================================================

install(TARGETS battery_warning_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# ヘッダーファイルがある場合はインストール設定を有効にします
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()