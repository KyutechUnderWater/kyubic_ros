cmake_minimum_required(VERSION 3.8)
project(planner_msgs)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(p_pid_controller_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
 "msg/WrenchPlan.msg"
 "action/PDLA.action"
 DEPENDENCIES p_pid_controller_msgs localization_msgs rclcpp_action
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
