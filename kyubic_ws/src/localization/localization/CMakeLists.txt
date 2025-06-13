cmake_minimum_required(VERSION 3.8)
project(localization)

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

# add libraries
## depth_odometry_component
add_library(depth_odometry_component SHARED
  ./src/depth_odometry_component.cpp
)

# target_link_libraries(depth_odometry_component
# )

target_include_directories(depth_odometry_component
  PRIVATE ./include/
)

ament_target_dependencies(depth_odometry_component
  rclcpp
  rclcpp_components
  driver_msgs
  localization_msgs
)

## imu_transform_component
ament_auto_add_library(imu_transform_component SHARED
  ./src/imu_transform_component.cpp
)

## dvl_odometry_component
ament_auto_add_library(dvl_odometry_component SHARED
  ./src/dvl_odometry_component.cpp
)

## localization_component
ament_auto_add_library(localization_component SHARED
  ./src/localization_component.cpp
)

# register components
rclcpp_components_register_node(depth_odometry_component
  PLUGIN "localization::DepthOdometry"
  EXECUTABLE depth_odometry_component_node
)

rclcpp_components_register_node(imu_transform_component
  PLUGIN "localization::IMUTransform"
  EXECUTABLE imu_transform_component_node
)

rclcpp_components_register_node(dvl_odometry_component
  PLUGIN "localization::DVLOdometry"
  EXECUTABLE dvl_odometry_component_node
)

rclcpp_components_register_node(localization_component
  PLUGIN "localization::Localization"
  EXECUTABLE localization_component_node
)

# set install config
install(TARGETS
  depth_odometry_component
  imu_transform_component
  dvl_odometry_component
  localization_component
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

ament_package()
