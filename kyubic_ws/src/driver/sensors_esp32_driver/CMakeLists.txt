cmake_minimum_required(VERSION 3.8)
project(sensors_esp32_driver)

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
add_protolink_message_from_ros_message("driver_msgs" "SystemStatus")
add_protolink_message_from_ros_message("driver_msgs" "PowerState")

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/button_battery_state.cpp
  src/buzzer_switch.cpp
  src/depth.cpp
  src/environment.cpp
  src/imu_reset.cpp
  src/leak.cpp
  src/rgb.cpp
  src/tilt_servo.cpp
  src/rtc_gnss.cpp
  src/rtc_time.cpp
  src/zed_power.cpp
)

target_link_libraries(${PROJECT_NAME}_component
  driver_msgs__ButtonBatteryState_proto
  driver_msgs__BoolStamped_proto
  driver_msgs__Int32Stamped_proto
  driver_msgs__BuzzerSwitch_proto
  driver_msgs__Environment_proto
  driver_msgs__RtcGnss_proto
  driver_msgs__RtcTime_proto
  driver_msgs__Depth_proto
)

# MACRO rclcpp_components_register_node
macro(register_components ClassName FileStem)
  set(node_name "${FileStem}_component_node")

  rclcpp_components_register_node(${PROJECT_NAME}_component
    PLUGIN "${PROJECT_NAME}::${ClassName}"
    EXECUTABLE ${node_name}
  )
endmacro()

register_components(ButtonBatteryState button_battery_state)
register_components(BuzzerSwitch       buzzer_switch)
register_components(Depth              depth)
register_components(Environment        environment)
register_components(ImuReset           imu_reset)
register_components(Leak               leak)
register_components(RGB                rgb)
register_components(RtcGnss            rtc_gnss)
register_components(RtcTime            rtc_time)
register_components(TiltServo          tilt_servo)
register_components(ZedPower           zed_power)

# set install config
install(TARGETS
  ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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
