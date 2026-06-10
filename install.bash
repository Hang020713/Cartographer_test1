#!/bin/bash

# This is the installation file for the fresh ubuntu 24.04 server/desktop LTS
# to install the whole requirement in order to use this workspace
# not verified on every device.
# Use at your own risks

# Get latest update and sudo permission
sudo apt update
echo "DONE 1st apt update"
sleep 1

# Install git & vcstool & colcon
sudo apt install -y git vcstool colcon
echo "DONE installing git, vcstool and colcon"
sleep 1

# Install SSH
sudo apt install openssh-server -y
echo "DONE installing openssh"
sleep 1
sudo systemctl enable --now ssh # Enable on boot
sudo ufw allow ssh # Allow firewall
echo "DONE setting up ssh"
sleep 1

# Install ROS2 Jazzy
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

# Check locale
sudo apt install locales
echo "DONE installing locales"
sleep 1
sudo locale-gen en_US en_US.UTF-8
sleep 1
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sleep 1
export LANG=en_US.UTF-8
echo "DONE setting up locales"
sleep 1

# Add the ROS 2 apt repository 
sudo apt install software-properties-common -y
echo "DONE installing software-properties-common"
sleep 1
sudo add-apt-repository universe -y
echo "DONE adding repo to apt"
sleep 1

# Install the ros2-apt-source package
sudo apt update
echo "DONE updating apt 2nd times"
sleep 1
sudo apt install curl -y
echo "DONE installing curl"
sleep 1
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
echo "DONE installing ros2 apt source"
sleep 1

# Install ROS2
sudo apt update
echo "DONE updateing apt 3rd times"
sleep 1

sudo apt install -y ros-jazzy-desktop
echo "DONE installing ros2 jazzy desktop"
sleep 1
sudo apt install -y ros-jazzy-ros-base
echo "DONE isntalling ros2 jazzy base"
sleep 1

# Source it
source /opt/ros/jazzy/setup.bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo "DONE sourcing ros2"
sleep 1

# Install mavros
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras
echo "DONE installing mavros"
sleep 1
ros2 run mavros install_geographiclib_datasets.sh
echo "DONE installing mavros's geographic lib"
sleep 1

# Git clone
cd ~/
git clone https://github.com/Hang020713/Cartographer_test1.git
echo "DONE cloning"
sleep 1

# Start building inside
cd Cartographer_test1
colcon build --symlink-install
echo "DONE building the Cartographer_test1"
sleep 1
source install/setup.bash
echo "DONE sourcing it"
sleep 1

# Install Micro XRCE-DDS Agent 
# https://docs.px4.io/main/en/middleware/uxrce_dds
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
echo "DONE cloning the Micro-XRCE-DDS-Agent"
sleep 1
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
echo "DONE making the Micro-XRCE-DDS-Agent"
sleep 1
sudo ldconfig /usr/local/lib/

echo "=====DONE All====="