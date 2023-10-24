cmake_minimum_required(VERSION 3.5)
project(transforms)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(custom_messages REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include ROS 2 packages
include_directories(include)

# Add your source files for 'paper'
set(PAPER_SRC_FILES
  src/paper.cpp
)

# Create the 'paper' executable
add_executable(paper ${PAPER_SRC_FILES})
ament_target_dependencies(paper
  rclcpp
  ros2_aruco_interfaces
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2
)

# Add your source files for 'static_camera_transform'
set(STATIC_CAMERA_TRANSFORM_SRC_FILES
  src/static_camera_transform.cpp
)

# Create the 'static_camera_transform' executable
add_executable(static_camera_transform ${STATIC_CAMERA_TRANSFORM_SRC_FILES})
ament_target_dependencies(static_camera_transform
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  std_msgs
  visualization_msgs
)

# Add your source files for 'path'
set(PATH_SRC_FILES
  src/path.cpp
)

# Create the 'path' executable
add_executable(path ${PATH_SRC_FILES})
ament_target_dependencies(path
  rclcpp
  geometry_msgs
  tf2_ros
  visualization_msgs
  std_msgs
  sensor_msgs
  ros2_aruco_interfaces
  rcl_interfaces
  custom_messages
)

# Install the executables
install(TARGETS
  paper
  static_camera_transform
  path
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()