cmake_minimum_required(VERSION 3.8)
project(wrench_action_sample)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(wrench_action_sample_msgs REQUIRED)

include_directories(include)

# C++ Executable
add_executable(wrench_action_server src/wrench_action_server.cpp)
ament_target_dependencies(wrench_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  wrench_action_sample_msgs
)

install(TARGETS
  wrench_action_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
