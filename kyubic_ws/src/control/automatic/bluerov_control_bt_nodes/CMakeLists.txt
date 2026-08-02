cmake_minimum_required(VERSION 3.8)
project(bluerov_control_bt_nodes)

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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(bluerov_control_bt_nodes SHARED
  ./src/check_buoy_detected.cpp
  ./src/check_pinger_pitch.cpp
  ./src/check_pinger_range.cpp
  ./src/waypoint_csv_writer.cpp
  ./src/buoy_detection_csv_reader.cpp
  ./src/write_hydrophone_waypoint_csv.cpp
  ./src/write_pinger_waypoint_csv.cpp
  ./src/write_axis_override_waypoint_csv.cpp
  ./src/check_buoy_in_frame.cpp
  ./src/check_buoy_position.cpp
  ./src/write_vision_waypoint_csv.cpp
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
