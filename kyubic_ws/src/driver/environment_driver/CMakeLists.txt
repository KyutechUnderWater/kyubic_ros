cmake_minimum_required(VERSION 3.8)
project(environment_driver)

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

# Protolinkメッセージ生成 ("Environment" 大文字)
add_protolink_message_from_ros_message("driver_msgs" "Environment")

include_directories(include)

# 実行ファイルの作成
add_executable(environment_driver src/environment_driver.cpp)

# 依存関係の設定
ament_target_dependencies(environment_driver
  timer
  rclcpp
  driver_msgs
  common_msgs
  protolink
)

# リンク設定
target_link_libraries(environment_driver
  driver_msgs__Environment_proto
)

install(TARGETS environment_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()