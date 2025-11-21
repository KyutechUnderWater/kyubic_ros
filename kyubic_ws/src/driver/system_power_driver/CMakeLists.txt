cmake_minimum_required(VERSION 3.8)
project(system_power_driver)

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
find_package(timer REQUIRED)
find_package(rclcpp REQUIRED)
find_package(driver_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(protolink REQUIRED)

# =========================================================
# Protolinkメッセージ生成 ("SystemPower" 大文字注意)
# =========================================================
add_protolink_message_from_ros_message("driver_msgs" "SystemPower")
# =========================================================

include_directories(include)

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    timer
    rclcpp
    driver_msgs
    common_msgs
    protolink
  )
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(system_power_driver)

# =========================================================
# リンク設定 (末尾に _proto)
# =========================================================
target_link_libraries(system_power_driver
  driver_msgs__SystemPower_proto
)
# =========================================================

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