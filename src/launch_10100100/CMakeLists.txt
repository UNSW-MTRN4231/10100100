cmake_minimum_required(VERSION 3.5)
project(launch_10100100)

find_package(ament_cmake REQUIRED)

# Install launch files
install( DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()