cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_messages REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)

# Add executable
add_executable(brain_node src/brain_node.cpp)

# Specify dependencies for the executable
ament_target_dependencies(brain_node
  rclcpp
  geometry_msgs
  tf2_msgs
  std_msgs
  custom_messages
)

# Declare the C++ executable
add_executable(move_command src/move_command.cpp)

target_include_directories(move_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_command PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


# Link the executable to the required libraries
ament_target_dependencies(move_command
  rclcpp
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_ros
  custom_messages
)

# Declare the C++ executable
add_executable(path_planning src/path_planning.cpp)

# Link the executable to the required libraries
ament_target_dependencies(path_planning
  rclcpp
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_ros
  custom_messages
)

# Install the executable
install(TARGETS 
  move_command
  brain_node
  path_planning
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
