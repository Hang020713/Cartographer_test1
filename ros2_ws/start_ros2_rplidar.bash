#!/bin/bash

# init ros2
source ~/init_ros2.bash

# mavros
sudo chmod 777 /dev/ttyACM0
gnome-terminal --tab --title="mavros" -- bash -c "ros2 run mavros mavros_node --ros-args -p fcu_url:=serial:///dev/ttyACM0:115200; exec bash"
sleep 5

# rplidar
sudo chmod 777 /dev/ttyUSB0
gnome-terminal --tab --title="rplidar" -- bash -c "ros2 launch sllidar_ros2 sllidar_c1_launch.py; exec bash"
sleep 5

# tf1
gnome-terminal --tab --title="tf1" -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame; exec bash"
# tf2
gnome-terminal --tab --title="tf2" -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link; exec bash"
sleep 5
# tf3
gnome-terminal --tab --title="tf3" -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link laser; exec bash"

#cartographer
gnome-terminal --tab --title="cartographer" -- bash -c "ros2 launch my_cartographer_config cartographer.launch.py; exec bash"
sleep 5

#rviz2
rviz2

