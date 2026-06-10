#!/bin/bash

# This is the installation file for the fresh ubuntu 24.04 server/desktop LTS
# to install the whole requirement in order to use this workspace
# not verified on every device.
# Use at your own risks

# Get latest update and sudo permission
sudo apt update

# Install git & vcstool & colcon
sudo apt install -y git vcstool colcon

# Install SSH
sudo apt update
sudo apt install openssh-server -y
sudo systemctl enable --now ssh # Enable on boot
sudo ufw allow ssh # Allow firewall

# Install ROS2 Jazzy
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt update

# Check locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add the ROS 2 apt repository 
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

# Install the ros2-apt-source package
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS2
sudo apt update
sudo apt install -y ros-jazzy-desktop
sudo apt install -y ros-jazzy-ros-base

# Source it
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

# Install mavros
sudo apt update
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras
ros2 run mavros install_geographiclib_datasets.sh

# Git clone
git clone https://github.com/Hang020713/Cartographer_test1.git

# Start building inside
cd Cartographer_test1
colcon build --symlink-install
source install/setup.bash


# Install Micro XRCE-DDS Agent 
# https://docs.px4.io/main/en/middleware/uxrce_dds
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

echo "DONE"