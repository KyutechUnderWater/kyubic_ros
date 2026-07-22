cmake_minimum_required(VERSION 3.8)
project(dvl75_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(blue_rov_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(common_msgs REQUIRED)
find_package(custom_socket REQUIRED)
find_package(driver_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(dvl75_parser src/dvl75_parser.cpp)
target_include_directories(dvl75_parser PUBLIC include)
set_target_properties(dvl75_parser PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(dvl75_driver_component SHARED src/dvl75_driver.cpp)
target_include_directories(dvl75_driver_component PRIVATE include)
target_link_libraries(dvl75_driver_component dvl75_parser)
ament_target_dependencies(dvl75_driver_component
  blue_rov_msgs
  builtin_interfaces
  common_msgs
  custom_socket
  driver_msgs
  geometry_msgs
  rclcpp
  rclcpp_components
)

rclcpp_components_register_node(dvl75_driver_component
  PLUGIN "dvl75_driver::Dvl75Driver"
  EXECUTABLE dvl75_driver
)

install(TARGETS dvl75_driver_component
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
  find_package(ament_cmake_clang_format QUIET)
  find_package(ament_cmake_gtest REQUIRED)

  if(ament_cmake_clang_format_FOUND)
    ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../../../.clang-format)
  endif()

  ament_add_gtest(test_dvl75_parser test/test_dvl75_parser.cpp)
  target_link_libraries(test_dvl75_parser dvl75_parser)
  target_include_directories(test_dvl75_parser PRIVATE include)
endif()

ament_package()
