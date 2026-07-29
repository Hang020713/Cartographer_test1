#!/bin/bash
# Source the core ROS2 environment
source ~/sgw-config
source /opt/ros/jazzy/setup.bash
source $SGW_WS/install/setup.bash
source $SGW_WS/sensor_monitor_node/install/setup.bash

# Execute the ROS2 nodes in background
ros2 run sensor_monitor sht3x_node &
PID1=$!
ros2 run sensor_monitor ina4230_node &
PID2=$!
ros2 run bms485_ros2 bms485_node &
PID3=$!

# Wait for both processes
wait $PID1 $PID2 $PID3
