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
find_package(status REQUIRED)
find_package(timer REQUIRED)
find_package(rclcpp REQUIRED)
find_package(driver_msgs REQUIRED)

include_directories(include)

# ament_auto_add_library(kyubic_interface SHARED
#   src/depth_comm_node.cpp
# )
function(custom_executable target)
  add_executable(${target} src/${target}.cpp src/bar30.cpp)
  ament_target_dependencies(${target}
    status
    serial
    timer
    rclcpp
    driver_msgs
  )
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(depth_driver)

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

