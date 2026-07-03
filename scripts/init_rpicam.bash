#!/bin/bash

sudo apt update && sudo apt upgrade -y
sudo apt install -y git clang meson ninja-build pkg-config libyaml-dev python3-yaml python3-ply python3-jinja2 openssl
sudo apt install -y libdw-dev libunwind-dev libudev-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libpython3-dev pybind11-dev libevent-dev libtiff-dev qt6-base-dev qt6-tools-dev-tools liblttng-ust-dev lttng-tools libexif-dev libjpeg-dev libgtest-dev abi-compliance-checker
sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev ffmpeg libavcodec-extra libavcodec-dev libavdevice-dev libpng-dev libpng-tools libepoxy-dev qt5-qmake qtmultimedia5-dev

# Clone the Raspberry Pi's libcamera repository
cd ~
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera    

# Configure the build
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled

# Compile and install
sudo ninja -C build install


# Go back to your home directory or another suitable location
cd ~

# Clone the rpicam-apps repository
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps/

# Configure the build. (Enable the features you need)
meson setup build -Denable_libav=disabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled

# Compile and install
meson compile -C build
sudo meson install -C build
sudo ldconfig

# Add user to group permission
sudo usermod -aG video $USER

# then reboot
echo "Please reboot your system to apply the changes. After reboot, you can test the camera using the rpicam-apps commands."