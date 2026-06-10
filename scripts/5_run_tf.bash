#!/bin/bash
# tf3
gnome-terminal --tab --title="tf3" -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link laser; exec bash"