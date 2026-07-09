#!/bin/bash
set -euo pipefail

# =============================================================================
# Installation script for fresh Ubuntu 24.04 LTS (server/desktop)
# Installs all requirements for this workspace
# Not verified on every device - use at your own risk
# =============================================================================

# Features: venv, ssh, ros2, mavros, uXRCE, pinctrl, rpicam, GPIO

# -------------------------------------------------------------------
# Configuration
# -------------------------------------------------------------------
ROS_DISTRO="jazzy"
WORKSPACE_NAME="Cartographer_test1"
WORKSPACE_REPO="https://github.com/Hang020713/Cartographer_test1.git"
MICRO_XRCE_VERSION="v2.4.3"
LOG_FILE="/tmp/install_$(date +%Y%m%d_%H%M%S).log"
BOOT_FIRMWARE="/boot/firmware"
PASSWORD="master"
HOTSPOT_ID="testingcm5"
HOTSPOT_PASSWORD="testingcm5"
HOTSPOT_IP="10.42.0.1"

# Comment out any of these to disable the feature
# PRE_INSTALL=1
# PYVENV_EN=1
# SSH_EN=1
# HOTSPOT_EN=1
# ROS2_EN=1
# MAVROS_EN=1
# CARTO_EN=1
# UXRCE_EN=1
# PINCTRL_EN=1
# RPICAM_EN=1
# GPIO_EN=1
# OVERLAY_EN=1
# MAVLINK_ROUTE_EN=1
# PWM_EN=1

# sudo ls -l first to get permission
echo "${PASSWORD}" | sudo ls -l 

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
if [ -n "${PRE_INSTALL+x}" ]; then
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
        i2c-tools \
        python3-pip \
        python3-venv
    log "Core build toolchain and utilities installed."
fi

# Create a virtual environment for python3 and source in ~/.bashrc
if [ -n "${PYVENV_EN+x}" ]; then
    python3 -m venv ~/.venv
    echo "" >> ~/.bashrc
    echo "# Auto-activate virtual environment" >> ~/.bashrc
    echo "source ~/.venv/bin/activate" >> ~/.bashrc
    echo "" >> ~/.bashrc
    echo "# Set ttyAMA0 permissions (consider using a udev rule instead)" >> ~/.bashrc
    echo "if [ -e /dev/ttyAMA0 ]; then" >> ~/.bashrc
    echo "    echo ${PASSWORD} | sudo -S chmod 666 /dev/ttyAMA0 2>/dev/null" >> ~/.bashrc
    echo "fi" >> ~/.bashrc
fi
source ~/.venv/bin/activate

# -------------------------------------------------------------------
# SSH setup
# -------------------------------------------------------------------
if [ -n "${SSH_EN+x}" ]; then
    apt_install openssh-server
    sudo systemctl enable --now ssh
    sudo ufw allow ssh
    log "SSH installed and configured."
fi

# Create hotspot connection
# Check if hotspot already exists
if [ -n "${HOTSPOT_EN+x}" ]; then
    # Create hotspot connection
    sudo nmcli connection add \
      type wifi \
      con-name Hotspot \
      autoconnect yes \
      wifi.mode ap \
      wifi.ssid ${HOTSPOT_ID} \
      ipv4.method shared \
      ipv4.addresses ${HOTSPOT_IP}/24
    
    # Set WiFi password
    sudo nmcli connection modify Hotspot \
      wifi-sec.key-mgmt wpa-psk \
      wifi-sec.psk "${HOTSPOT_PASSWORD}"
    
    log "Hotspot configured."

    # Enable the hotspot
    sudo nmcli connection up Hotspot || log_warning "Failed to start hotspot"
fi

# -------------------------------------------------------------------
# ROS2 Jazzy installation
# Reference: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
# -------------------------------------------------------------------
if [ -n "${ROS2_EN+x}" ]; then
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
        python3-lark
    pip3 install colcon-common-extensions vcstool catkin-pkg rosdep lark numpy
    log "ROS2 build tools installed."

    # Initialize rosdep
    sudo rosdep init || true
    rosdep update
    log "rosdep initialized."

    # Install ROS2 base
    apt_install "ros-${ROS_DISTRO}-ros-base"
    log "ROS2 ${ROS_DISTRO} base installed."
    sleep 1

    apt_install \
        "ros-${ROS_DISTRO}-cartographer-ros" \
        "ros-${ROS_DISTRO}-cartographer-ros-msgs"
    log "Cartographer ROS dependencies installed."

    # Source ROS2
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
    log "ROS2 sourced."
    sleep 1
fi

# -------------------------------------------------------------------
# MAVROS installation
# -------------------------------------------------------------------
if [ -n "${MAVROS_EN+x}" ]; then
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
fi

# -------------------------------------------------------------------
# Clone and build Cartographer workspace
# -------------------------------------------------------------------
if [ -n "${CARTO_EN+x}" ]; then
    cd ~
    if [ ! -d "$WORKSPACE_NAME" ]; then
        git clone "$WORKSPACE_REPO"
        log "Cloned $WORKSPACE_NAME repository."
    else
        log "$WORKSPACE_NAME already exists, pulling latest..."
        cd "$WORKSPACE_NAME"
        git pull
    fi

    cd ~/"$WORKSPACE_NAME"
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    log "$WORKSPACE_NAME built successfully."
    
    set +u
    source install/setup.bash
    set -u
fi

# -------------------------------------------------------------------
# Micro XRCE-DDS Agent installation
# Reference: https://docs.px4.io/main/en/middleware/uxrce_dds
# -------------------------------------------------------------------
if [ -n "${UXRCE_EN+x}" ]; then
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
fi

# -------------------------------------------------------------------
# Raspberry Pi utilities (pinctrl)
# -------------------------------------------------------------------
if [ -n "${PINCTRL_EN+x}" ]; then
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
fi


# -------------------------------------------------------------------
# rpicam installation
# -------------------------------------------------------------------
if [ -n "${RPICAM_EN+x}" ]; then
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

    pip3 install ply
    pip3 install pyyaml
    pip3 install jinja2

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

    sudo usermod -aG video "$USER"
    log "rpicam installed."
fi

# -------------------------------------------------------------------
# GPIO setup
# -------------------------------------------------------------------
if [ -n "${GPIO_EN+x}" ]; then
    apt_install gpiod libgpiod-dev python3-libgpiod python3-pip python3-gpiozero python3-lgpio
    pip3 install gpiozero lgpio
    sudo usermod -aG dialout "$USER"
    log "GPIO tools installed."
fi

# echo "dtoverlay=vc4-kms-v3d,cma-512" | sudo tee -a "${BOOT_FIRMWARE}/config.txt"
# echo "gpu_mem=128" | sudo tee -a "${BOOT_FIRMWARE}/config.txt"

# -------------------------------------------------------------------
# Device tree overlays for camera & PWM
# -------------------------------------------------------------------
if [ -n "${OVERLAY_EN+x}" ]; then
    sudo cp ~/${WORKSPACE_NAME}/dtoverlays/imx708-cam0.dtbo /boot/firmware/overlays/imx708-cam0.dtbo
    sudo cp ~/${WORKSPACE_NAME}/dtoverlays/imx708-cam1.dtbo /boot/firmware/overlays/imx708-cam1.dtbo
    sudo cp ~/${WORKSPACE_NAME}/dtoverlays/pwm-pi5.dtbo /boot/firmware/overlays/pwm-pi5.dtbo

    # Replace the /boot/firmware/config.txt
    sudo rm /boot/firmware/config.txt
    sudo cp ~/${WORKSPACE_NAME}/config.txt /boot/firmware/config.txt

    sudo rm /boot/firmware/cmdline.txt
    sudo cp ~/${WORKSPACE_NAME}/cmdline.txt /boot/firmware/cmdline.txt

    log "dtoverlay done."
fi

# Mavlink_router
if [ -n "${MAVLINK_ROUTE_EN+x}" ]; then
    sudo apt-get install git meson ninja-build gcc g++ pkg-config systemd

    cd ~
    git clone https://github.com/mavlink-router/mavlink-router.git
    cd mavlink-router
    git submodule update --init --recursive

    meson setup build .
    ninja -C build
    sudo ninja -C build install
    
    sudo mkdir -p /etc/mavlink-router
    sudo cp ~/${WORKSPACE_NAME}/main.conf /etc/mavlink-router/main.conf

    log "Mavlink router done."
fi


# -------------------------------------------------------------------
# Cleanup and finalization
# -------------------------------------------------------------------
enable_apt_services

log "===== Installation Complete ====="
log "Please reboot your system or log out and back in for group changes to take effect."