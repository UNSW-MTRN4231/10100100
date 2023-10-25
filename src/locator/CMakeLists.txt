cmake_minimum_required(VERSION 3.5)
project(locator)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# Add your source files for 'paper'
set(PAPER_SRC_FILES
  src/paper.cpp
)

# Create the 'paper' executable
add_executable(paper ${PAPER_SRC_FILES})
ament_target_dependencies(paper
  rclcpp
  ros2_aruco_interfaces
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
  tf2_ros
  tf2
  geometry_msgs
)

# Install the executables
install(TARGETS
  paper
  static_camera_transform
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()