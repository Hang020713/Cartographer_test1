#!/bin/bash
set -euo pipefail

# =============================================================================
# Installation script for fresh Ubuntu 24.04 LTS (server/desktop)
# Installs all requirements for this workspace
# Not verified on every device - use at your own risk
# =============================================================================

# -------------------------------------------------------------------
# Configuration
# -------------------------------------------------------------------
ROS_DISTRO="jazzy"
WORKSPACE_NAME="Cartographer_test1"
WORKSPACE_REPO="https://github.com/Hang020713/Cartographer_test1.git"
MICRO_XRCE_VERSION="v2.4.3"
LOG_FILE="/tmp/install_$(date +%Y%m%d_%H%M%S).log"
BOOT_FIRMWARE="/boot/firmware"

# -------------------------------------------------------------------
# Helper functions
# -------------------------------------------------------------------
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

log_error() {
    log "ERROR: $*" >&2
}

log_warning() {
    log "WARNING: $*"
}

wait_for_apt() {
    log "Checking for apt/dpkg locks..."
    while sudo fuser /var/lib/dpkg/lock-frontend \
                     /var/lib/dpkg/lock \
                     /var/lib/apt/lists/lock \
                     /var/cache/apt/archives/lock >/dev/null 2>&1; do
        log "  Lock held by another process. Waiting 5s..."
        sleep 5
    done
    log "  Lock is free."
}

apt_install() {
    wait_for_apt
    sudo apt-get install -y "$@"
}

apt_update() {
    wait_for_apt
    sudo apt-get update
}

disable_apt_services() {
    log "Disabling automatic apt services temporarily..."
    sudo systemctl stop unattended-upgrades.service 2>/dev/null || true
    sudo systemctl stop apt-daily.service apt-daily.timer 2>/dev/null || true
    sudo systemctl stop apt-daily-upgrade.service apt-daily-upgrade.timer 2>/dev/null || true
}

enable_apt_services() {
    log "Re-enabling automatic apt services..."
    sudo systemctl start apt-daily.timer apt-daily-upgrade.timer 2>/dev/null || true
    sudo systemctl start unattended-upgrades.service 2>/dev/null || true
}

# -------------------------------------------------------------------
# Start installation
# -------------------------------------------------------------------
log "Starting installation process..."

# Stop background apt services to prevent lock contention
disable_apt_services

# Initial system update
apt_update
wait_for_apt
sudo apt-get full-upgrade -y
log "System updated successfully."

# Install core build tools and utilities
apt_install \
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
    lsb-release \
    network-manager \
    i2c-tools
log "Core build toolchain and utilities installed."

# -------------------------------------------------------------------
# SSH setup
# -------------------------------------------------------------------
apt_install openssh-server
sudo systemctl enable --now ssh
sudo ufw allow ssh
log "SSH installed and configured."

# -------------------------------------------------------------------
# ROS2 Jazzy installation
# Reference: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
# -------------------------------------------------------------------
log "Setting up ROS2 ${ROS_DISTRO}..."

# Configure locale
apt_install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
log "Locale configured."

# Add ROS2 repository
apt_install software-properties-common
sudo add-apt-repository universe -y
apt_update

# Install ROS2 apt source package
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
UBUNTU_CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
ROS_APT_DEB="ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb"
curl -L -o "/tmp/${ROS_APT_DEB}" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/${ROS_APT_DEB}"

wait_for_apt
sudo dpkg -i "/tmp/${ROS_APT_DEB}"
log "ROS2 apt source installed."

# Update and install ROS tools
apt_update
apt_install \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-catkin-pkg \
    python3-rosdep \
    python3-pip \
    python3-lark
# pip3 install colcon-common-extensions
# pip3 install vcstool
# pip3 install catkin-pkg
# pip3 install rosdep
log "ROS2 build tools installed."

# Initialize rosdep
sudo rosdep init || true
rosdep update
log "rosdep initialized."

# Install ROS2 base
apt_install "ros-${ROS_DISTRO}-ros-base"
log "ROS2 ${ROS_DISTRO} base installed."

# Source ROS2
source "/opt/ros/${ROS_DISTRO}/setup.bash"
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
log "ROS2 sourced."

# -------------------------------------------------------------------
# MAVROS installation
# -------------------------------------------------------------------
apt_install "ros-${ROS_DISTRO}-mavros" "ros-${ROS_DISTRO}-mavros-extras"
log "MAVROS installed."

# Install geographiclib datasets
GEO_SCRIPT="$(ros2 pkg prefix mavros 2>/dev/null)/lib/mavros/install_geographiclib_datasets.sh"
if [ ! -f "$GEO_SCRIPT" ]; then
    GEO_SCRIPT="$(find "/opt/ros/${ROS_DISTRO}" -name install_geographiclib_datasets.sh 2>/dev/null | head -n 1)"
fi

if [ -f "$GEO_SCRIPT" ]; then
    sudo bash "$GEO_SCRIPT"
    log "MAVROS geographiclib datasets installed."
else
    log_error "install_geographiclib_datasets.sh not found!"
fi

# -------------------------------------------------------------------
# Cartographer ROS dependencies
# -------------------------------------------------------------------
apt_install \
    "ros-${ROS_DISTRO}-cartographer-ros" \
    "ros-${ROS_DISTRO}-cartographer-ros-msgs"
log "Cartographer ROS dependencies installed."

# -------------------------------------------------------------------
# Raspberry Pi utilities (pinctrl)
# -------------------------------------------------------------------
apt_install cmake git device-tree-compiler build-essential libncurses5-dev libncursesw5-dev libfdt-dev

cd ~
if [ ! -d "utils" ]; then
    git clone https://github.com/raspberrypi/utils.git
fi

cd utils
cmake . && make && sudo make install

cd pinctrl
cmake . && make && sudo make install
log "pinctrl installed."

# -------------------------------------------------------------------
# rpicam installation
# -------------------------------------------------------------------
sudo apt-get update && sudo apt-get upgrade -y

apt_install \
    git clang meson ninja-build pkg-config \
    libyaml-dev openssl \
    libdw-dev libunwind-dev libudev-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libpython3-dev pybind11-dev libevent-dev libtiff-dev \
    qt6-base-dev qt6-tools-dev-tools \
    liblttng-ust-dev lttng-tools libexif-dev libjpeg-dev \
    libgtest-dev abi-compliance-checker \
    cmake libboost-program-options-dev libdrm-dev ffmpeg \
    libavcodec-extra libavcodec-dev libavdevice-dev \
    libpng-dev libpng-tools libepoxy-dev \
    qt5-qmake qtmultimedia5-dev \
    python3-yaml python3-ply python3-jinja2

# pip3 install ply
# pip3 install pyyaml
# pip3 install jinja2

# Build libcamera
cd ~
if [ ! -d "libcamera" ]; then
    git clone https://github.com/raspberrypi/libcamera.git
fi

cd libcamera
meson setup build --buildtype=release \
    -Dpipelines=rpi/vc4,rpi/pisp \
    -Dipas=rpi/vc4,rpi/pisp \
    -Dv4l2=true \
    -Dgstreamer=enabled \
    -Dtest=false \
    -Dlc-compliance=disabled \
    -Dcam=disabled \
    -Dqcam=disabled \
    -Ddocumentation=disabled \
    -Dpycamera=enabled
sudo ninja -C build install

# Build rpicam-apps
cd ~
if [ ! -d "rpicam-apps" ]; then
    git clone https://github.com/raspberrypi/rpicam-apps.git
fi

cd rpicam-apps
meson setup build \
    -Denable_libav=disabled \
    -Denable_drm=enabled \
    -Denable_egl=enabled \
    -Denable_qt=enabled \
    -Denable_opencv=disabled \
    -Denable_tflite=disabled \
    -Denable_hailo=disabled
meson compile -C build
sudo meson install -C build
sudo ldconfig
log "rpicam installed."

# -------------------------------------------------------------------
# GPIO setup
# -------------------------------------------------------------------
apt_install gpiod libgpiod-dev python3-libgpiod python3-pip python3-gpiozero python3-lgpio
# pip3 install libgpiod gpiozero lgpio
sudo usermod -aG dialout "$USER"
log "GPIO tools installed."

# -------------------------------------------------------------------
# User permissions
# -------------------------------------------------------------------
sudo usermod -aG video "$USER"
log "User added to video and dialout groups."

echo "dtoverlay=vc4-kms-v3d,cma-512" | sudo tee -a "${BOOT_FIRMWARE}/config.txt"
echo "gpu_mem=128" | sudo tee -a "${BOOT_FIRMWARE}/config.txt"

# -------------------------------------------------------------------
# Device tree overlays for camera
# -------------------------------------------------------------------
OVERLAYS_SRC="${HOME}/${WORKSPACE_NAME}/dtoverlays"
OVERLAYS_DEST="${BOOT_FIRMWARE}/overlays"

if [ -d "$OVERLAYS_SRC" ]; then
    # Ensure overlay destination exists
    sudo mkdir -p "$OVERLAYS_DEST"
    
    # Copy device tree overlays
    for overlay in imx708-cam0.dtbo imx708-cam1.dtbo; do
        if [ -f "${OVERLAYS_SRC}/${overlay}" ]; then
            sudo cp "${OVERLAYS_SRC}/${overlay}" "${OVERLAYS_DEST}/${overlay}"
            log "Copied ${overlay} to ${OVERLAYS_DEST}"
        else
            log_warning "${overlay} not found in ${OVERLAYS_SRC}"
        fi
    done

    # Append camera configuration to config files
    CONFIG_FILE="${BOOT_FIRMWARE}/config.txt"
    USERCFG_FILE="${BOOT_FIRMWARE}/usercfg.txt"

    # Check if entries already exist before appending
    if ! grep -q "camera_auto_detect=0" "$USERCFG_FILE" 2>/dev/null; then
        echo "camera_auto_detect=0" | sudo tee -a "$USERCFG_FILE" > /dev/null
        log "Disabled camera auto-detect in usercfg.txt"
    else
        log "Camera auto-detect already disabled in usercfg.txt"
    fi

    if ! grep -q "dtoverlay=imx708-cam0" "$CONFIG_FILE" 2>/dev/null; then
        echo "dtoverlay=imx708-cam0" | sudo tee -a "$CONFIG_FILE" > /dev/null
        log "Added imx708-cam0 overlay to config.txt"
    else
        log "imx708-cam0 overlay already in config.txt"
    fi

    if ! grep -q "dtoverlay=imx708-cam1" "$CONFIG_FILE" 2>/dev/null; then
        echo "dtoverlay=imx708-cam1" | sudo tee -a "$CONFIG_FILE" > /dev/null
        log "Added imx708-cam1 overlay to config.txt"
    else
        log "imx708-cam1 overlay already in config.txt"
    fi

    log "Device tree overlays configured."
else
    log_warning "Overlay source directory not found: ${OVERLAYS_SRC}"
    log_warning "Make sure ${WORKSPACE_NAME} repository includes dtoverlays directory"
fi

# -------------------------------------------------------------------
# Clone and build Cartographer workspace
# -------------------------------------------------------------------
cd ~
if [ ! -d "$WORKSPACE_NAME" ]; then
    git clone "$WORKSPACE_REPO"
    log "Cloned $WORKSPACE_NAME repository."
else
    log "$WORKSPACE_NAME already exists, pulling latest..."
    cd "$WORKSPACE_NAME"
    git pull
fi

cd "$WORKSPACE_NAME"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
log "$WORKSPACE_NAME built successfully."

source install/setup.bash

# -------------------------------------------------------------------
# Micro XRCE-DDS Agent installation
# Reference: https://docs.px4.io/main/en/middleware/uxrce_dds
# -------------------------------------------------------------------
cd ~
if [ ! -d "Micro-XRCE-DDS-Agent" ]; then
    git clone -b "$MICRO_XRCE_VERSION" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
fi

# Verify compiler availability
if ! command -v g++ >/dev/null 2>&1; then
    log_error "g++ not found. Installing build-essential..."
    apt_install build-essential cmake
fi

cd Micro-XRCE-DDS-Agent
mkdir -p build && cd build
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig /usr/local/lib/
log "Micro XRCE-DDS Agent installed."

# -------------------------------------------------------------------
# Cleanup and finalization
# -------------------------------------------------------------------
enable_apt_services

log "===== Installation Complete ====="
log "Please reboot your system or log out and back in for group changes to take effect."