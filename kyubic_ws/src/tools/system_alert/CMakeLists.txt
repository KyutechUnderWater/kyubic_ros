cmake_minimum_required(VERSION 3.8)
project(system_alert)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(ALSA REQUIRED)

add_library(localization_error_component SHARED
  ./src/localization_error.cpp
)

target_include_directories(localization_error_component
  PRIVATE ./include/
)

ament_target_dependencies(localization_error_component
  rclcpp
  rclcpp_components
  localization_msgs
)

target_link_libraries(localization_error_component
  ${ALSA_LIBRARIES}
)

rclcpp_components_register_node(localization_error_component
  PLUGIN "tools::system_alert::LocalizationError"
  EXECUTABLE localization_error_component_node
)

install(TARGETS
  localization_error_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  # config
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
