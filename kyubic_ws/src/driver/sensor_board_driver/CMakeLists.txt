cmake_minimum_required(VERSION 3.8)
project(sensor_board_driver)

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

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_protolink_message_from_ros_message("driver_msgs" "Int32Stamped")
add_protolink_message_from_ros_message("driver_msgs" "BoolStamped")
add_protolink_message_from_ros_message("driver_msgs" "ButtonBatteryState")
add_protolink_message_from_ros_message("driver_msgs" "BuzzerSwitch")
add_protolink_message_from_ros_message("driver_msgs" "Environment")
add_protolink_message_from_ros_message("driver_msgs" "RtcGnss")
add_protolink_message_from_ros_message("driver_msgs" "RtcTime")
add_protolink_message_from_ros_message("driver_msgs" "Depth")

ament_auto_add_executable(button_battery_state ./src/button_battery_state.cpp)
ament_auto_add_executable(imu_reset ./src/imu_reset.cpp)
ament_auto_add_executable(leak ./src/leak.cpp)
ament_auto_add_executable(depth ./src/depth.cpp)
ament_auto_add_executable(rgb ./src/rgb.cpp)
ament_auto_add_executable(tilt_servo ./src/tilt_servo.cpp)
ament_auto_add_executable(zed_power ./src/zed_power.cpp)
ament_auto_add_executable(buzzer_switch ./src/buzzer_switch.cpp)
ament_auto_add_executable(environment ./src/environment.cpp)
ament_auto_add_executable(rtc_gnss ./src/rtc_gnss.cpp)
ament_auto_add_executable(rtc_time ./src/rtc_time.cpp)
target_link_libraries(button_battery_state driver_msgs__ButtonBatteryState_proto)
target_link_libraries(imu_reset driver_msgs__BoolStamped_proto)
target_link_libraries(leak driver_msgs__BoolStamped_proto)
target_link_libraries(depth driver_msgs__Depth_proto)
target_link_libraries(rgb driver_msgs__Int32Stamped_proto)
target_link_libraries(tilt_servo driver_msgs__Int32Stamped_proto)
target_link_libraries(zed_power driver_msgs__BoolStamped_proto)
target_link_libraries(buzzer_switch driver_msgs__BuzzerSwitch_proto)
target_link_libraries(environment driver_msgs__Environment_proto)
target_link_libraries(rtc_gnss driver_msgs__RtcGnss_proto)
target_link_libraries(rtc_time driver_msgs__RtcTime_proto)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
