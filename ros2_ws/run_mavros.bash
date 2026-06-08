source /opt/ros/jazzy/setup.bash
sudo chmod 777 /dev/ttyACM0
ros2 run mavros mavros_node --ros-args -p fcu_url:=serial:///dev/ttyACM0:115200
