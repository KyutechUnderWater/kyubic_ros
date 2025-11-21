cmake_minimum_required(VERSION 3.8)
project(driver_msgs)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/BatteryWarning.msg"
	"msg/BuzzerSwitch.msg"
	"msg/Depth.msg"
	"msg/DVL.msg"
	"msg/Environment.msg"
	"msg/Gnss.msg"
	"msg/IMU.msg"
	"msg/LED.msg"
	"msg/PowerState.msg"
	"msg/PowerSwitch.msg"
	"msg/RtcGnss.msg"
	"msg/RtcTime.msg"
	"msg/SystemPower.msg"
	"msg/Thruster.msg"

	"srv/Command.srv"
	DEPENDENCIES std_msgs
	DEPENDENCIES geometry_msgs
	DEPENDENCIES sensor_msgs
	DEPENDENCIES common_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

