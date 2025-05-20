cmake_minimum_required(VERSION 3.8)
project(depth_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()
find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(driver_msgs REQUIRED)


# bar30
add_library(bar30 SHARED
  src/bar30.cpp
)

target_include_directories(bar30
  PRIVATE include
)

ament_target_dependencies(bar30
  serial
)

# depth_driver_component
add_library(depth_driver_component SHARED
  src/depth_driver_component.cpp
)

target_include_directories(depth_driver_component
  PRIVATE include
)

target_link_libraries(depth_driver_component
  bar30
)

ament_target_dependencies(depth_driver_component
  rclcpp
  rclcpp_components
  driver_msgs
)

# depth_odometry_component
add_library(depth_odometry_component SHARED
  src/depth_odometry_component.cpp
)

target_include_directories(depth_odometry_component
  PRIVATE include
)

ament_target_dependencies(depth_odometry_component
  rclcpp
  rclcpp_components
  driver_msgs
)

# Register components
rclcpp_components_register_node(depth_driver_component
  PLUGIN "depth_driver::DepthDriver"
  EXECUTABLE depth_driver_component_node
)

rclcpp_components_register_node(depth_odometry_component
  PLUGIN "depth_driver::DepthOdometry"
  EXECUTABLE depth_odometry_component_node
)


# # ament_auto_add_executable(depth_driver)
# function(custom_executable target)
#   add_executable(${target} src/${target}.cpp src/bar30.cpp)
#   ament_target_dependencies(${target}
#     serial
#     rclcpp
#     rclcpp_components
#     driver_msgs
#   )
#   install(TARGETS ${target}
#     DESTINATION lib/${PROJECT_NAME})
# endfunction()
#
# include_directories(include)
#
# custom_executable(depth_driver)

# install settings
install(TARGETS
  depth_driver_component
  depth_odometry_component
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

ament_package()

