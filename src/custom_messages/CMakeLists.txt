cmake_minimum_required(VERSION 3.5)
project(custom_messages)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define the custom message files
set(msg_files
  "msg/RobotAction.msg" 
  "msg/ArucoMessage.msg"
  "srv/PathClient.srv"
)

# Generate ROS 2 custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()