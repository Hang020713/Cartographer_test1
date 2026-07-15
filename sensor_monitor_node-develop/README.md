# Sensor Node for ROS 2

The `sensor_monitor` ROS 2 package for reading data from INA4230 current sensors and SHT3x temperature/humidity sensors.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Workspace Setup & Build](#workspace-setup--build)
- [I2C Device Configuration](#i2c-device-configuration)
- [Running the Nodes](#running-the-nodes)
- [Parameters](#parameters)
- [Topics](#topics)
- [Troubleshooting](#troubleshooting)

## Prerequisites

| Component    | Version / Details                       |
| ------------ | --------------------------------------- |
| Hardware     | Raspberry Pi CM5 + SCH_CM5 Carrier Board |
| OS           | Ubuntu 22.04                            |
| ROS 2        | Jazzy Jalisco                           |
| Build tool   | `colcon`                                |
| I2C bus      | `/dev/i2c-3` (default; must be enabled and accessible) |

### Install ROS 2 and colcon

```bash
# Install ROS 2 Jazzy (full desktop install)
sudo apt update && sudo apt install ros-jazzy-desktop

# Install colcon build tool
sudo apt install python3-colcon-common-extensions
```

## Workspace Setup & Build

### 1. Create the workspace directory (customizable name)

```bash
mkdir -p ~/boat_control_ws/src
```

### 2. Clone the package

Place the `sensor_monitor` package into the workspace `src` directory:

```bash
cd ~/boat_control_ws/src/
git clone ssh://git@stlgit.seasongroup.com:10022/squaredog/seal/sensor_monitor_node.git
```

The final directory structure should be:

```text
boat_control_ws/
└── src/
    └── sensor_monitor/
        ├── CMakeLists.txt
        ├── package.xml
        └── src/
            ├── ina4230_node.cpp
            └── sht3x_node.cpp
```

### 3. Install dependencies

```bash
cd ~/boat_control_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the workspace

```bash
cd ~/boat_control_ws
colcon build --symlink-install
```

> The `--symlink-install` flag uses symbolic links so that changes to scripts or configuration files take effect without rebuilding.

A successful build outputs:

```text
Summary: 1 package finished [X.Xs]
```

### 5. Source the workspace environment

```bash
source ~/boat_control_ws/install/setup.bash
```

> **Tip:** Add this line to `~/.bashrc` to automatically load the environment in every new terminal:
>
> ```bash
> echo "source ~/boat_control_ws/install/setup.bash" >> ~/.bashrc
> ```

## I2C Device Configuration

### Raspberry Pi CM5 I2C-3 Bus Setup

```bash
sudo nano /boot/firmware/config.txt
```

```ini
[all]
# dtparam=spi=on

[all]
dtoverlay=i2c3-pi5,pins_6_7
```

```bash
sudo reboot
```

### Enable the I2C Bus

```bash
# List available I2C buses
ls /dev/i2c-*

# If i2c-3 does not appear, enable the I2C interface
sudo raspi-config           # Raspberry Pi
# or
sudo i2cdetect -l           # list all I2C buses
```

### Grant User Access to I2C

```bash
sudo usermod -aG i2c $USER
# Re-login for the change to take effect
```

### Detect Sensor Devices

```bash
# Scan I2C bus 3 for device addresses
sudo i2cdetect -y 3
```

Expected results:

| Sensor  | I2C Address                    |
| ------- | ------------------------------ |
| INA4230 | 0x40, 0x41, 0x44 (configurable) |
| SHT3x   | 0x45 (default)                  |

## Running the Nodes

Ensure you have [sourced the environment](#5-source-the-workspace-environment) before running the commands below.

### Start the SHT3x Temperature/Humidity Node

```bash
ros2 run sensor_monitor sht3x_node
# or
ros2 run sensor_monitor sht3x_node --ros-args --log-level debug
```

Example output:

```text
[INFO] [sht3x_node]: SHT3x Node has started. Reading from 0x45
[INFO] [sht3x_node]: Published -> Temp: 28.53 °C, Hum: 65.21 %
```

### Start the INA4230 Current Sensor Node

```bash
ros2 run sensor_monitor ina4230_node
# or
ros2 run sensor_monitor ina4230_node --ros-args --log-level debug
```

Example output:

```text
[INFO] [ina4230_node]: Detected INA4230 at I2C address 0x40
[INFO] [ina4230_node]: Detected INA4230 at I2C address 0x41
[INFO] [ina4230_node]: Detected INA4230 at I2C address 0x44
[INFO] [ina4230_node]: INA4230 node startup successful; reading 3 sensor devices...
```

### Run Both Nodes Concurrently

```bash
ros2 run sensor_monitor sht3x_node &
ros2 run sensor_monitor ina4230_node
```

## Parameters

### SHT3x Node

| Parameter     | Type   | Default       | Description                   |
| ------------- | ------ | ------------- | ----------------------------- |
| `i2c_device`  | string | `/dev/i2c-3`  | I2C bus device path           |
| `i2c_address` | int    | `0x45`        | SHT3x sensor I2C address      |

Override parameters at runtime:

```bash
ros2 run sensor_monitor sht3x_node --ros-args -p i2c_device:=/dev/i2c-1 -p i2c_address:=0x44
```

### INA4230 Node

| Parameter          | Type     | Default                    | Description                                |
| ------------------ | -------- | -------------------------- | ------------------------------------------ |
| `i2c_device`       | string   | `/dev/i2c-3`               | I2C bus device path                        |
| `i2c_addresses`    | int[]    | `[0x40, 0x41, 0x44]`      | List of INA4230 sensor I2C addresses       |
| `current_lsb`      | double   | `0.001`                    | Current LSB (minimum resolution, in A)     |
| `shunt_resistance`  | double   | `0.01`                     | Shunt resistor value (in Ω)                |

Override parameters at runtime:

```bash
ros2 run sensor_monitor ina4230_node --ros-args \
  -p i2c_device:=/dev/i2c-1 \
  -p i2c_addresses:=[0x40,0x41] \
  -p current_lsb:=0.0005 \
  -p shunt_resistance:=0.02
```

## Topics

### SHT3x Node

| Topic           | Type                                   | Description                |
| --------------- | -------------------------------------- | -------------------------- |
| `~/temperature` | `sensor_msgs/msg/Temperature`          | Temperature data (°C)      |
| `~/humidity`    | `sensor_msgs/msg/RelativeHumidity`     | Relative humidity (0.0–1.0) |

Monitor temperature:

```bash
ros2 topic echo /sht3x_node/temperature
```

### INA4230 Node

Each sensor has 4 channels. Topic names follow this pattern:

```text
ina4230_0x<address>/channel_<number>
```

| Example Topic               | Type                    | Description                         |
| --------------------------- | ----------------------- | ----------------------------------- |
| `ina4230_0x40/channel_1`   | `std_msgs/msg/Float64`  | Sensor 0x40, channel 1 current (A)  |
| `ina4230_0x40/channel_2`   | `std_msgs/msg/Float64`  | Sensor 0x40, channel 2 current (A)  |
| `ina4230_0x41/channel_1`   | `std_msgs/msg/Float64`  | Sensor 0x41, channel 1 current (A)  |
| …                           |                         |                                     |

Monitor a current topic:

```bash
ros2 topic echo /ina4230_0x40/channel_1
```

List all active topics:

```bash
ros2 topic list
```

## Troubleshooting

### 1. "Unable to open the I2C bus device"

Check whether the I2C bus is enabled and the user has access:

```bash
ls -l /dev/i2c-3
groups $USER | grep i2c
```

### 2. "No INA4230 devices found on bus"

- Verify that the sensors are correctly wired (SDA, SCL, VCC, GND).
- Use `i2cdetect -y 3` to confirm the device address.
- Check that the `i2c_addresses` parameter matches the actual address.

### 3. "CRC Checksum Failed"

SHT3x data validation failure. Common causes:

- Unstable I2C connection or excessively long wiring.
- Power supply noise.
- Try increasing the `kMeasWait` delay (currently 20 ms).

### 4. Build fails with "ament_cmake not found"

Make sure ROS 2 is installed and sourced:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/boat_control_ws && colcon build --symlink-install
```
