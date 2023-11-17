gnome-terminal -e 'source install/setup.bash'

# setupRealur5e
gnome-terminal -t "DriverServer" -e 'ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 use_fake_hardware:=false initial_joint_controller:=joint_trajectory_controller launch_rviz:=false'

sleep 10

gnome-terminal -t "MoveitServer" -e 'ros2 launch ur_moveit_config ur_moveit.launch.py robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true'


# cammera
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0"


# launch everything
ros2 launch launch_10100100 launch_all.launch.py


