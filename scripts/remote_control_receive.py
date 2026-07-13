import remote_control_utils as rc_utils
from mavlink_controller import MavController
import time
import threading

# Functions Parameters
INPUT_PORT=None  # Serial port to be selected by the user
INPUT_BAUDRATE=None  # Baud rate for the serial communication
END_CHAR='\n'  # End character for the command
MAVLINK_SERIAL_PORT = "/dev/tty.usbmodem1201"
MAVLINK_SERIAL_BAUD = 115200

# PWM Parameters
THROTTLE_RAW_MIN = 0
THROTTLE_RAW_MAX = 255
THROTTLE_RAW_CENTER = 127
THROTTLE_PWM_MIN = 1100
THROTTLE_PWM_MAX = 1900
THROTTLE_PWM_CENTER = 1500
SERVO_LEFT_CHANNEL=3
SERVO_RIGHT_CHANNEL=5
STEERING_RAW_MIN = 0
STEERING_RAW_MAX = 255
STEERING_RAW_CENTER = 127
STEERING_PWM_MIN = 500 
STEERING_PWM_MAX = 2500
STEERING_PWM_CENTER = 1500

# Current system status
MODES = ["IDLE", "PRE-WASH", "AUTO", "MANUAL"]
MODES_STATUS = ["FAILED", "SUCCESS"]

def raw_to_percent(
    raw: int,
    raw_min: int,
    raw_max: int,
    raw_center: int,
) -> int:
    raw = max(raw_min, min(raw_max, raw))

    if raw < raw_center:
        if raw_center <= raw_min:
            return 0
        return int(round(-100 + ((raw - raw_min) / (raw_center - 1 - raw_min)) * 99))

    if raw == raw_center:
        return 0

    if raw_center >= raw_max:
        return 100

    return int(round(1 + ((raw - (raw_center + 1)) / (raw_max - (raw_center + 1))) * 99))

def percent_to_pwm(
    pct: int,
    pwm_min: int,
    pwm_max: int,
    pwm_center: int,
) -> int:
    pct = max(-100, min(100, pct))
    if pct < 0:
        return int(round(pwm_center + (pct / 100.0) * (pwm_center - pwm_min)))
    return int(round(pwm_center + (pct / 100.0) * (pwm_max - pwm_center)))

# Main Function
if __name__ == "__main__":
    # Select baud rate, default to 4800 if not provided
    try:
        INPUT_BAUDRATE = int(input("Enter baud rate (default 4800): ") or 4800)
    except ValueError:
        print("Invalid baud rate. Using default 4800.")
        INPUT_BAUDRATE = 4800

    # Select the serial port
    INPUT_PORT = rc_utils.select_serial_port(INPUT_PORT)
    print(f"Selected port: {INPUT_PORT}")

    if INPUT_PORT is None:
        print("No port selected. Exiting.")
        raise SystemExit(1)

    # Init serial connection
    ser = rc_utils.init_serial_connection(INPUT_PORT, INPUT_BAUDRATE)
    if ser is None:
        raise SystemExit(1)

    # Init MavController first
    print("Initializing MavController...")
    mav_controller = MavController(port=MAVLINK_SERIAL_PORT, baud=MAVLINK_SERIAL_BAUD)
    mav_master = mav_controller.get_master()
    
    while not mav_controller.is_connected:
        print("Waiting for connection...")
        time.sleep(1)

    # Arm the vehicle
    if not mav_controller.is_armed:
        print("Arming the vehicle...")
        mav_controller.arm()
        time.sleep(1)  # Wait for a moment to ensure the vehicle is armed

    # Configure the device
    response = rc_utils.send_config_command(ser, end_char=END_CHAR)
    print(f"Response: {response}\n-EOF")
    if "OK" in response:
        print("Configuration command sent successfully.")
    else:
        print("Configuration command failed or returned unexpected response.")
        ser.close() # Close connection before exiting
        raise SystemExit(1)

    # Start receiving
    try:
        while True:
            # LX, LY, RX, RY
            if ser.in_waiting > 0:
                received_data = ser.read(8)
                print(f"Received data: {received_data.hex()}\n- EOF")
                time.sleep(0.01)

                # Parse the received data into PWM values (unit: 0-255)
                steering_left = received_data[0:1]  # LX
                throttle_left = received_data[1:2]  # LY
                steering_right = received_data[2:3] # RX
                throttle_right = received_data[3:4] # RY
                print(f"Steering Left: {steering_left.hex()}\nThrottle Left: {throttle_left.hex()}\nSteering Right: {steering_right.hex()}\nThrottle Right: {throttle_right.hex()}\n-EOF")

                # Calculate PWM values based on the received data
                throttle_raw_left = int.from_bytes(throttle_left, byteorder='little')
                throttle_raw_right = int.from_bytes(throttle_right, byteorder='little')
                throttle_left_pct = raw_to_percent(throttle_raw_left, THROTTLE_RAW_MIN, THROTTLE_RAW_MAX, THROTTLE_RAW_CENTER)
                throttle_right_pct = raw_to_percent(throttle_raw_right, THROTTLE_RAW_MIN, THROTTLE_RAW_MAX, THROTTLE_RAW_CENTER)
                throttle_left_pwm = percent_to_pwm(throttle_left_pct, THROTTLE_PWM_MIN, THROTTLE_PWM_MAX, THROTTLE_PWM_CENTER)
                throttle_right_pwm = percent_to_pwm(throttle_right_pct, THROTTLE_PWM_MIN, THROTTLE_PWM_MAX, THROTTLE_PWM_CENTER)

                print(f"throttle_left_pct={throttle_left_pct} throttle_right_pct={throttle_right_pct}")
                print(f"throttle_left_pwm={throttle_left_pwm} throttle_right_pwm={throttle_right_pwm}")

                # Send the mapped raw PWM values directly
                mav_master.mav.rc_channels_override_send(
                    mav_master.target_system,
                    mav_master.target_component,
                    throttle_left_pwm,     # chan1
                    0,                # chan2
                    throttle_right_pwm,     # chan3
                    0,                # chan4
                    0,                # chan5
                    0,                # chan6
                    0,                # chan7
                    0                 # chan8
                )

                # Set servo positions based on the received data
                steering_raw_left = int.from_bytes(steering_left, byteorder='little')
                steering_raw_right = int.from_bytes(steering_right, byteorder='little')
                steering_left_pct = raw_to_percent(steering_raw_left, STEERING_RAW_MIN, STEERING_RAW_MAX, STEERING_RAW_CENTER)
                steering_right_pct = raw_to_percent(steering_raw_right, STEERING_RAW_MIN, STEERING_RAW_MAX, STEERING_RAW_CENTER)
                steering_left_pwm = percent_to_pwm(steering_left_pct, STEERING_PWM_MIN, STEERING_PWM_MAX, STEERING_PWM_CENTER)
                steering_right_pwm = percent_to_pwm(steering_right_pct, STEERING_PWM_MIN, STEERING_PWM_MAX, STEERING_PWM_CENTER)
                print(f"steering_left_pwm={steering_left_pwm} steering_right_pwm={steering_right_pwm}")

                # Set servo
                mav_controller.set_servo(SERVO_LEFT_CHANNEL, steering_left_pwm)
                mav_controller.set_servo(SERVO_RIGHT_CHANNEL, steering_right_pwm)
                print(f"Set servo positions: Left={steering_left_pwm}, Right={steering_right_pwm}\n-EOF")

    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        ser.close()  # Close connection before exiting
