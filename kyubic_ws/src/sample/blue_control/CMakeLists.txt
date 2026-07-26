cmake_minimum_required(VERSION 3.8)
project(blue_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(driver_msgs REQUIRED)
find_package(blue_rov_msgs REQUIRED)
find_package(localization_msgs REQUIRED)

include_directories(include)

add_executable(blue_control_listener src/blue_control_listener.cpp)
ament_target_dependencies(blue_control_listener
  rclcpp
  driver_msgs
  blue_rov_msgs
  localization_msgs
)

install(TARGETS
  blue_control_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
