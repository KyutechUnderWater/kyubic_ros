cmake_minimum_required(VERSION 3.8)
project(projection_dynamic_look_ahead_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
ament_auto_find_build_dependencies()

# add library
ament_auto_add_library(pdla_planner_component SHARED
  ./src/pdla_planner.cpp
)
target_link_libraries(pdla_planner_component
  Eigen3::Eigen
)

# register components
rclcpp_components_register_node(pdla_planner_component
  PLUGIN "planner::PDLAPlanner"
  EXECUTABLE pdla_planner_component_node
)

# set install config
install(TARGETS
  pdla_planner_component
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
