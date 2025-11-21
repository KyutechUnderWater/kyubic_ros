cmake_minimum_required(VERSION 3.8)
project(buzzer_switch_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(driver_msgs REQUIRED)
find_package(protolink REQUIRED)

# メッセージ生成
add_protolink_message_from_ros_message("driver_msgs" "BuzzerSwitch")

include_directories(include)

add_executable(buzzer_switch_driver src/buzzer_switch_driver.cpp)
ament_target_dependencies(buzzer_switch_driver
  rclcpp
  driver_msgs
  protolink
)

# リンク設定
target_link_libraries(buzzer_switch_driver
  driver_msgs__BuzzerSwitch_proto
)

install(TARGETS buzzer_switch_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()