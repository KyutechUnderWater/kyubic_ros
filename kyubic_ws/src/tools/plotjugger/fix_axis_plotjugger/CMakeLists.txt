cmake_minimum_required(VERSION 3.8)
project(fix_axis_plotjugger)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# library
## add fix_axis_plotjugger_component
ament_auto_add_library(fix_axis_plotjugger_component SHARED
  ./src/fix_axis_plotjugger.cpp
)


# register components
rclcpp_components_register_node(fix_axis_plotjugger_component
  PLUGIN "tools::plotjugger::FixAxisPlotjugger"
  EXECUTABLE fix_axis_plotjugger_component_node
)


# set install config
install(TARGETS
  fix_axis_plotjugger_component
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

ament_auto_package()
