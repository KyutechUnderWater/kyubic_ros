cmake_minimum_required(VERSION 3.8)
project(actuator_rp2040_driver)

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
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_protolink_message_from_ros_message("driver_msgs" "Actuator")

ament_auto_add_library(${PROJECT_NAME}_component SHARED ./src/actuator_rp2040_driver.cpp)
target_link_libraries(${PROJECT_NAME}_component driver_msgs__Actuator_proto)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "actuator_rp2040_driver::ActuatorRP2040"
  EXECUTABLE ${PROJECT_NAME}_component_node
)

# set install config
install(TARGETS
  ${PROJECT_NAME}_component
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

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
