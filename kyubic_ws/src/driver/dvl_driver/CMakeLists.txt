cmake_minimum_required(VERSION 3.8)
project(dvl_driver)

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

ament_auto_add_library(path_finder SHARED
  src/path_finder.cpp
)

ament_auto_add_library(dvl_driver_component SHARED
  src/dvl_driver_component.cpp
)

ament_auto_add_library(dvl_odometry_component SHARED
  src/dvl_odometry_component.cpp
)
# dvl_driver_component
# add_library(dvl_driver_component SHARED
#   # src/path_finder.cpp
#   src/dvl_driver_component.cpp
# )
#
# target_link_libraries(dvl_driver_component
#   path_finder
# )
#
# target_include_directories(dvl_driver_component
#   PRIVATE include
# )
#
# ament_target_dependencies(dvl_driver_component
#   rclcpp
#   rclcpp_components
#   driver_msgs
# )

# Register components
rclcpp_components_register_node(dvl_driver_component
  PLUGIN "dvl_driver::DVLDriver"
  EXECUTABLE dvl_driver_component_node
)

rclcpp_components_register_node(dvl_odometry_component
  PLUGIN "dvl_driver::DVLOdometry"
  EXECUTABLE dvl_odometry_component_node
)

# install settings
install(TARGETS
  dvl_driver_component
  dvl_odometry_component
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

ament_auto_package()
