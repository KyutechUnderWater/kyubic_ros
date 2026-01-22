cmake_minimum_required(VERSION 3.8)
project(wrench_planner)

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

# add library
ament_auto_add_library(wrench_planner_component SHARED
  ./src/wrench_planner.cpp
)

ament_auto_add_library(velocity_wrench_planner_component SHARED
  ./src/velocity_wrench_planner.cpp
)

# register components
rclcpp_components_register_node(wrench_planner_component
  PLUGIN "planner::WrenchPlanner"
  EXECUTABLE wrench_planner_component_node
)

rclcpp_components_register_node(velocity_wrench_planner_component
  PLUGIN "planner::VelocityWrenchPlanner"
  EXECUTABLE velocity_wrench_planner_component_node
)

# set install config
install(TARGETS
  wrench_planner_component
  velocity_wrench_planner_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

##############################################
# --- Diagnostics Plugin ---
ament_auto_add_library(system_health_check_plugins SHARED
  ./src-check/velocity_wrench_planner_check.cpp
  ./src-check/wrench_planner_check.cpp
)

# Install settings for pluginlib
install(TARGETS system_health_check_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(system_health_check plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
