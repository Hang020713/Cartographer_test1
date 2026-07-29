# -*- coding: utf-8 -*-

SERIAL_PORT = "/dev/ttyAMA0" # /dev/ttyUSB0
SERIAL_BAUD = 115200

FLIGHT_MODES = ["MANUAL", "GUIDED"]

PAGE_TITLE = "ArduPilot Boat Remote Control"
PAGE_ICON = "🚢"
LAYOUT = "wide"
SIDEBAR_STATE = "collapsed"

# Control constants
SERVO_LEFT_CHANNEL=6
SERVO_RIGHT_CHANNEL=7

THRUSTER_PWM_CENTER = 1500

BRUSH_LEFT_CHANNEL=4
BRUSH_RIGHT_CHANNEL=8
BRUSH_PWM_CENTER = 1500

STEERING_LEFT_PWM = 1850
STEERING_RIGHT_PWM = 1150 
STEERING_CENTER_PWM = 1500

# Manual motor control speed offsets (PWM offset from center 1500)
MANUAL_FORWARD_SPEED = 100      # Both motors forward: PWM = 1500 + 100 = 1600
MANUAL_BACKWARD_SPEED = -100    # Both motors backward: PWM = 1500 - 100 = 1400
MANUAL_TURN_SPEED = 100         # Turning: active motor PWM offset

# BMS (Battery Management System) constants
BMS_PORT = "/dev/ttyAMA3"       # BMS Modbus RTU serial port (separate from flight controller)
BMS_BAUD = 9600          # Modbus RTU baudrate
BMS_POLL_INTERVAL = 5    # Polling interval in seconds

# ── Sensor Monitor ───────────────────────────────────────────────
# I2C bus
SENSOR_I2C_DEVICE = "/dev/i2c-3"
SENSOR_SHT3X_ADDR = 0x45
SENSOR_INA4230_ADDRS = [0x40, 0x41, 0x44]
SENSOR_POLL_INTERVAL = 2.0  # Sensor data refresh interval (seconds)

# Level transmitter (Modbus RTU / RS-485)
LEVEL_PORT = "/dev/ttyAMA3"
LEVEL_BAUD = 9600
LEVEL_SLAVE_ID = 2

# Current channel definitions: (i2c_addr, channel_number, pwm_label, rated_current_a, display_name)
SENSOR_CURRENT_CHANNELS = [
    # Left side (0x41)
    (0x41, 3, "pwm0", 6.0, "Left Propulsion Motor"),
    (0x41, 4, "pwm4", 6.0, "Left Auxiliary Motor"),
    (0x41, 1, "pwm5", 4.0, "Left Brush Motor"),
    (0x41, 2, "pwm7", 4.0, "Left Servo"),
    # Right side (0x40 + 0x44)
    (0x40, 4, "pwm1", 6.0, "Right Propulsion Motor"),
    (0x40, 3, "pwm2", 6.0, "Right Auxiliary Motor"),
    (0x40, 2, "pwm3", 4.0, "Right Brush Motor"),
    (0x44, 1, "pwm6", 4.0, "Right Servo"),
]

MSG_WARN_CONNECTION_REQUIRED = "Please connect and arm first!"
