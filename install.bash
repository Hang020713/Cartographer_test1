#!/bin/bash

# This is the installation file for the fresh ubuntu 24.04 server/desktop LTS
# to install the whole requirement in order to use this workspace
# not verified on every device.
# Use at your own risks

# ---------------------------------------------------------------
# Wait for apt/dpkg locks to be released (avoids race with
# unattended-upgrades / apt-daily background services)
# ---------------------------------------------------------------
wait_for_apt() {
    echo "Checking for apt/dpkg locks..."
    while sudo fuser /var/lib/dpkg/lock-frontend \
                     /var/lib/dpkg/lock \
                     /var/lib/apt/lists/lock \
                     /var/cache/apt/archives/lock >/dev/null 2>&1; do
        echo "  Another process holds the apt/dpkg lock. Waiting 5s..."
        sleep 5
    done
    echo "  Lock is free."
}

# Stop background apt services to prevent lock contention
echo "Disabling automatic apt services temporarily..."
sudo systemctl stop unattended-upgrades.service 2>/dev/null || true
sudo systemctl stop apt-daily.service apt-daily.timer 2>/dev/null || true
sudo systemctl stop apt-daily-upgrade.service apt-daily-upgrade.timer 2>/dev/null || true
sleep 1

# Get latest update and full upgrade (full-upgrade resolves version
# conflicts like the libbz2 / bzip2 mismatch)
wait_for_apt
sudo apt update
wait_for_apt
sudo apt full-upgrade -y
echo "DONE 1st apt update + full-upgrade"
sleep 1

wait_for_apt
sudo apt install -y \
    build-essential \
    cmake \
    g++ \
    git \
    pkg-config \
    curl \
    wget \
    vim \
    nano \
    htop \
    net-tools \
    unzip \
    software-properties-common \
    ca-certificates \
    gnupg \
    lsb-release
echo "DONE installing core build toolchain + common utilities"
sleep 1

# Install core build toolchain EARLY (needed for cmake/make builds later)
# This prevents the "No CMAKE_CXX_COMPILER could be found" error.
wait_for_apt
sudo apt install -y build-essential cmake g++ git
echo "DONE installing build-essential, cmake, g++, git"
sleep 1

# Install SSH
wait_for_apt
sudo apt install -y openssh-server
echo "DONE installing openssh"
sleep 1
sudo systemctl enable --now ssh # Enable on boot
sudo ufw allow ssh # Allow firewall
echo "DONE setting up ssh"
sleep 1

# Install ROS2 Jazzy
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

# Check locale
wait_for_apt
sudo apt install -y locales
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
wait_for_apt
sudo apt install -y software-properties-common
echo "DONE installing software-properties-common"
sleep 1
wait_for_apt
sudo add-apt-repository universe -y
echo "DONE adding universe repo to apt"
sleep 1

# Install the ros2-apt-source package
wait_for_apt
sudo apt update
echo "DONE updating apt 2nd times"
sleep 1
wait_for_apt
sudo apt install -y curl
echo "DONE installing curl"
sleep 1
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
wait_for_apt
sudo dpkg -i /tmp/ros2-apt-source.deb
echo "DONE installing ros2 apt source"
sleep 1

# Update apt now that universe + ROS repos are available
wait_for_apt
sudo apt update
echo "DONE updating apt 3rd times"
sleep 1

# Install build tools (correct package names, AFTER repos are added)
wait_for_apt
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-catkin-pkg \
    python3-rosdep \
    python3-pip
echo "DONE installing colcon, vcstool, catkin-pkg, rosdep"
sleep 1

# Initialise rosdep (safe to ignore error if already initialised)
sudo rosdep init || true
rosdep update
echo "DONE setting up rosdep"
sleep 1

# Install ROS2
wait_for_apt
sudo apt install -y ros-jazzy-desktop
echo "DONE installing ros2 jazzy desktop"
sleep 1
wait_for_apt
sudo apt install -y ros-jazzy-ros-base
echo "DONE installing ros2 jazzy base"
sleep 1

# Source it
source /opt/ros/jazzy/setup.bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo "DONE sourcing ros2"
sleep 1

# Install mavros
wait_for_apt
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras
echo "DONE installing mavros"
sleep 1

# Install mavros's geographiclib datasets (needs root).
# We run the script directly with 'sudo bash' instead of 'ros2 run',
# because sudo does not preserve the sourced ROS environment.
GEO_SCRIPT="$(ros2 pkg prefix mavros)/lib/mavros/install_geographiclib_datasets.sh"
if [ -f "$GEO_SCRIPT" ]; then
    sudo bash "$GEO_SCRIPT"
else
    GEO_SCRIPT="$(find /opt/ros/jazzy -name install_geographiclib_datasets.sh 2>/dev/null | head -n 1)"
    if [ -n "$GEO_SCRIPT" ]; then
        sudo bash "$GEO_SCRIPT"
    else
        echo "WARNING: install_geographiclib_datasets.sh not found!"
    fi
fi
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
cd ~/
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
echo "DONE cloning the Micro-XRCE-DDS-Agent"
sleep 1

# Sanity check: make sure a C++ compiler is available before building
if ! command -v g++ >/dev/null 2>&1; then
    echo "ERROR: g++ not found. Installing build-essential..."
    wait_for_apt
    sudo apt install -y build-essential cmake
fi

cd Micro-XRCE-DDS-Agent
mkdir -p build
cd build
cmake ..
make
sudo make install
echo "DONE making the Micro-XRCE-DDS-Agent"
sleep 1
sudo ldconfig /usr/local/lib/

# Re-enable background apt services
echo "Re-enabling automatic apt services..."
sudo systemctl start apt-daily.timer apt-daily-upgrade.timer 2>/dev/null || true
sudo systemctl start unattended-upgrades.service 2>/dev/null || true

echo "=====DONE All====="