#!/bin/bash
# Source the core ROS2 environment
source ~/sgw-config
source /opt/ros/jazzy/setup.bash
source $SGW_WS/install/setup.bash
source $SGW_WS/sensor_monitor_node/install/setup.bash

$VENV_PATH/bin/python3 $SGW_WS/onboard_scripts/remote_control_receive.py
