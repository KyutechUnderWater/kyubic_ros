cmake_minimum_required(VERSION 3.8)
project(power_switch_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(driver_msgs REQUIRED)
find_package(protolink REQUIRED)

# =========================================================
# Protolink メッセージ生成 ("PowerSwitch" 大文字注意)
# =========================================================
add_protolink_message_from_ros_message("driver_msgs" "PowerSwitch")
# =========================================================

include_directories(include)

add_executable(power_switch_driver src/power_switch_driver.cpp)
ament_target_dependencies(power_switch_driver
  rclcpp
  driver_msgs
  protolink
)

# =========================================================
# リンク設定 (_proto を忘れずに)
# =========================================================
target_link_libraries(power_switch_driver
  driver_msgs__PowerSwitch_proto
)
# =========================================================

install(TARGETS power_switch_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

ament_package()