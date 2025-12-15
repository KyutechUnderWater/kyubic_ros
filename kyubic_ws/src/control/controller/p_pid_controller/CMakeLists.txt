cmake_minimum_required(VERSION 3.8)
project(p_pid_controller)

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
## p_pid_controller
ament_auto_add_library(p_pid_controller SHARED
  ./src/p_pid_controller.cpp
)
# ament_autoがyaml-cppを検出しないため，手動で追記
target_link_libraries(p_pid_controller yaml-cpp)

## test_pid_component
ament_auto_add_library(test_p_pid_component SHARED
  ./src/test_p_pid.cpp
)

## measure_parameter_component
ament_auto_add_library(measure_parameter_component SHARED
  ./src/measure_parameter.cpp
)

rclcpp_components_register_node(test_p_pid_component
  PLUGIN "controller::TestPPID"
  EXECUTABLE test_p_pid_component_node
)

rclcpp_components_register_node(measure_parameter_component
  PLUGIN "controller::MeasureParam"
  EXECUTABLE measure_parameter_component_node
)


# set install config
install(TARGETS
  test_p_pid_component
  measure_parameter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
