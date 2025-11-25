cmake_minimum_required(VERSION 3.8)
project(behavior_tree)

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

ament_auto_add_executable(${PROJECT_NAME}
  ./src/bt_executor.cpp
  ./src/bt_nodes/check_sensors_status.cpp
  ./src/bt_nodes/sample/fibonacci_action.cpp
  ./src/bt_nodes/sample/wrench_action.cpp
  ./src/bt_nodes/always_running.cpp
  ./src/bt_nodes/lifecycle_manager.cpp
  ./src/bt_nodes/update_mode.cpp
  ./src/bt_nodes/waypoint_action.cpp
  ./src/bt_nodes/qr_action.cpp
)

install(DIRECTORY
  bt_xml
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
