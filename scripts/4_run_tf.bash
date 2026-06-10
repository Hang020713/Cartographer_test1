#!/bin/bash
# tf2
gnome-terminal --tab --title="tf2" -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link; exec bash"