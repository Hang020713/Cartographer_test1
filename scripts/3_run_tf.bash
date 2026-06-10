#!/bin/bash
# tf1
gnome-terminal --tab --title="tf1" -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame; exec bash"