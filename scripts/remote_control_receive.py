import remote_control_utils as rc_utils
from mavlink_controller import MavController
import time
import threading
import queue
import light_utils

HAVE_PIXHAWK = False

# Functions Parameters
INPUT_PORT=None  # Serial port to be selected by the user
INPUT_BAUDRATE=None  # Baud rate for the serial communication
END_CHAR='\n'  # End character for the command
#MAVLINK_SERIAL_PORT = "/dev/tty.usbmodem1201"
MAVLINK_SERIAL_PORT = 'tcp:127.0.0.1:5760'
MAVLINK_SERIAL_BAUD = 115200

# PWM Parameters
THROTTLE_RAW_MIN = 0
THROTTLE_RAW_MAX = 255
THROTTLE_RAW_CENTER = 127
THROTTLE_PWM_MIN = 1000
THROTTLE_PWM_MAX = 2000
THROTTLE_PWM_CENTER = 1500

SERVO_LEFT_CHANNEL=3
SERVO_RIGHT_CHANNEL=5
STEERING_RAW_MIN = 0
STEERING_RAW_MAX = 255
STEERING_RAW_CENTER = 127
STEERING_PWM_MIN = 500 
STEERING_PWM_MAX = 2500
STEERING_PWM_CENTER = 1500

BRUSH_PWM_MIN = 1000
BRUSH_PWM_MAX = 2000
BRUSHED_PWM_CENTER = 1500
BRUSH_LEFT_CHANNEL=4
BRUSH_RIGHT_CHANNEL=8

PWM_PIN_MAP = {
    "18": ("2", "a3"),
    "19": ("3", "a3"),
}
UVC_LIGHT_PIN = "19"
UVC_LIGHT_FREQ = 2000
UVC_LIGHT_LAST_DUTY = 0

# Payload parameter
MESSAGE_ID = 0xAA
ID = 0x00

# Current system status
current_mode = rc_utils.MODES.MANUAL
current_mode_status = rc_utils.MODE_STATUS.ONGOING

# Command queue
command_queue = queue.Queue()

# Threads
program_stop_event = threading.Event()
command_handler_thread = None

def map_brush_pwm(brush_dir, brush_speed, side):
    brush_dir_raw = int.from_bytes(brush_dir, byteorder='little')
    brush_speed_raw = int.from_bytes(brush_speed, byteorder='little')

    brush_speed_pct = brush_speed_raw
    speed_magnitude = abs(brush_speed_pct) / 100.0

    if brush_dir_raw == 0x00:
        brush_pwm = BRUSHED_PWM_CENTER
    elif brush_dir_raw == 0x01:
        if side == "left":
            brush_pwm = int(BRUSHED_PWM_CENTER + (speed_magnitude * (BRUSH_PWM_MAX - BRUSHED_PWM_CENTER)))
        else:
            brush_pwm = int(BRUSHED_PWM_CENTER - (speed_magnitude * (BRUSHED_PWM_CENTER - BRUSH_PWM_MIN)))
    elif brush_dir_raw == 0x02:
        if side == "left":
            brush_pwm = int(BRUSHED_PWM_CENTER - (speed_magnitude * (BRUSHED_PWM_CENTER - BRUSH_PWM_MIN)))
        else:
            brush_pwm = int(BRUSHED_PWM_CENTER + (speed_magnitude * (BRUSH_PWM_MAX - BRUSHED_PWM_CENTER)))
    else:
        brush_pwm = BRUSHED_PWM_CENTER

    return int(brush_pwm)

def command_handler_thread_func():
    while not program_stop_event.is_set():
        try:
            next_command = command_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        
        print(f"[{time.time()}][Command Handler] Parse next command: {next_command}")
        id = received_data[0:1]
        command_type = rc_utils.get_command_type(received_data[1:2])
        
        # Parse command type
        match command_type:
            case rc_utils.COMMANDS.REQUEST_STATUS:
                print("Got request status")

                # Send the status
                byte_data = bytes([MESSAGE_ID, ID, current_mode, current_mode_status])
                response = rc_utils.send_bytes(ser, byte_data, read_response=False)
                
            case rc_utils.COMMANDS.MANUAL_CONTROL:
                print("Got manual control")

                # Parse the reading
                steering_left = received_data[2:3]  # LX
                throttle_left = received_data[3:4]  # LY
                steering_right = received_data[4:5] # RX
                throttle_right = received_data[5:6] # RY

                brush_dir = received_data[6:7]
                brush_speed = received_data[7:8]
                light_pct = received_data[8:9]

                print(f"[{time.time()}]Steering Left: {steering_left.hex()}\nThrottle Left: {throttle_left.hex()}\nSteering Right: {steering_right.hex()}\nThrottle Right: {throttle_right.hex()}\n-EOF")
                print(f"[{time.time()}]Brush Dir: {brush_dir}, {brush_speed}")
                print(f"[{time.time()}]Light: {light_pct}")

                update_manual_control(steering_left, throttle_left, steering_right, throttle_right, brush_dir, brush_speed, light_pct)
        time.sleep(0.01)


def update_manual_control(steering_left, throttle_left, steering_right, throttle_right, brush_dir, brush_speed, light_pct):
    global UVC_LIGHT_LAST_DUTY

    # Calculate PWM values based on the received data
    throttle_raw_left = int.from_bytes(throttle_left, byteorder='little')
    throttle_raw_right = int.from_bytes(throttle_right, byteorder='little')
    throttle_left_pct = rc_utils.raw_to_percent(throttle_raw_left, THROTTLE_RAW_MIN, THROTTLE_RAW_MAX, THROTTLE_RAW_CENTER)
    throttle_right_pct = rc_utils.raw_to_percent(throttle_raw_right, THROTTLE_RAW_MIN, THROTTLE_RAW_MAX, THROTTLE_RAW_CENTER)
    throttle_left_pwm = rc_utils.percent_to_pwm(throttle_left_pct, THROTTLE_PWM_MIN, THROTTLE_PWM_MAX, THROTTLE_PWM_CENTER)
    throttle_right_pwm = rc_utils.percent_to_pwm(throttle_right_pct, THROTTLE_PWM_MIN, THROTTLE_PWM_MAX, THROTTLE_PWM_CENTER)

    # Calculate PWM values based on the received data
    throttle_raw_left = int.from_bytes(throttle_left, byteorder='little')
    throttle_raw_right = int.from_bytes(throttle_right, byteorder='little')
    throttle_left_pct = rc_utils.raw_to_percent(throttle_raw_left, THROTTLE_RAW_MIN, THROTTLE_RAW_MAX, THROTTLE_RAW_CENTER)
    throttle_right_pct = rc_utils.raw_to_percent(throttle_raw_right, THROTTLE_RAW_MIN, THROTTLE_RAW_MAX, THROTTLE_RAW_CENTER)
    throttle_left_pwm = rc_utils.percent_to_pwm(throttle_left_pct, THROTTLE_PWM_MIN, THROTTLE_PWM_MAX, THROTTLE_PWM_CENTER)
    throttle_right_pwm = rc_utils.percent_to_pwm(throttle_right_pct, THROTTLE_PWM_MIN, THROTTLE_PWM_MAX, THROTTLE_PWM_CENTER)

    print(f"throttle_left_pct={throttle_left_pct} throttle_right_pct={throttle_right_pct}")
    print(f"throttle_left_pwm={throttle_left_pwm} throttle_right_pwm={throttle_right_pwm}")

    # Set servo positions based on the received data
    steering_raw_left = int.from_bytes(steering_left, byteorder='little')
    steering_raw_right = int.from_bytes(steering_right, byteorder='little')
    steering_left_pct = rc_utils.raw_to_percent(steering_raw_left, STEERING_RAW_MIN, STEERING_RAW_MAX, STEERING_RAW_CENTER)
    steering_right_pct = rc_utils.raw_to_percent(steering_raw_right, STEERING_RAW_MIN, STEERING_RAW_MAX, STEERING_RAW_CENTER)
    steering_left_pwm = rc_utils.percent_to_pwm(steering_left_pct, STEERING_PWM_MIN, STEERING_PWM_MAX, STEERING_PWM_CENTER)
    steering_right_pwm = rc_utils.percent_to_pwm(steering_right_pct, STEERING_PWM_MIN, STEERING_PWM_MAX, STEERING_PWM_CENTER)
    print(f"steering_left_pwm={steering_left_pwm} steering_right_pwm={steering_right_pwm}")
    print(f"Set servo positions: Left={steering_left_pwm}, Right={steering_right_pwm}\n-EOF")

    # Brush
    left_brush_pwm = map_brush_pwm(brush_dir, brush_speed, "left")
    right_brush_pwm = map_brush_pwm(brush_dir, brush_speed, "right")
    print(f"[{time.time()}]Left Brush: {left_brush_pwm}, Right Brush: {right_brush_pwm}")

    # Light
    uvc_current_duty = int.from_bytes(light_pct, byteorder='little')
    print(f"[{time.time()}]UVC light: {uvc_current_duty}")
    if UVC_LIGHT_LAST_DUTY != uvc_current_duty:
        light_utils.mode_freq(UVC_LIGHT_PIN, PWM_PIN_MAP[UVC_LIGHT_PIN][0], PWM_PIN_MAP[UVC_LIGHT_PIN][1], UVC_LIGHT_FREQ)
        light_utils.mode_duty(UVC_LIGHT_PIN, PWM_PIN_MAP[UVC_LIGHT_PIN][0], uvc_current_duty)
    UVC_LIGHT_LAST_DUTY = uvc_current_duty

    if HAVE_PIXHAWK:
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

        # Set servo
        mav_controller.set_servo(SERVO_LEFT_CHANNEL, steering_left_pwm)
        mav_controller.set_servo(SERVO_RIGHT_CHANNEL, steering_right_pwm)

        # Set Brushed
        mav_controller.set_servo(BRUSH_LEFT_CHANNEL, left_brush_pwm)
        mav_controller.set_servo(BRUSH_RIGHT_CHANNEL, right_brush_pwm)

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
    
    # Configure the device
    response = rc_utils.send_config_command(ser, end_char=END_CHAR)
    print(f"Response: {response}\n-EOF")
    if "OK" in response:
        print("Configuration command sent successfully.")
    else:
        print("Configuration command failed or returned unexpected response.")
        ser.close() # Close connection before exiting
        raise SystemExit(1)

    # Init MavController
    if HAVE_PIXHAWK:
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
    
    # Init threads
    program_stop_event.clear()
    if command_handler_thread is None or not command_handler_thread.is_alive():
        command_handler_thread = threading.Thread(target=command_handler_thread_func, daemon=True)
        command_handler_thread.start()
    print("lora receive thread started and will keep running.")

    # Parse the command
    try:
        while True:
            # Receive lora thread
            received_data = rc_utils.read_frame(ser, MESSAGE_ID, rc_utils.INQUERY_PAYLOAD_LEN)
            if received_data is None:
                continue

            # Really received new command
            # print("[Lora] Received new command")
            # print(received_data)

            # Parse data
            id = received_data[0:1]
            command_type = received_data[1:2]
            # print(f"id: {id}")
            # print(f"command type: {command_type}, {rc_utils.get_command_type(command_type).name}")

            # Added to command queue
            command_queue.put(received_data)
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        program_stop_event.set()
        if command_handler_thread is not None:
            command_handler_thread.join(timeout=1)
        ser.close()  # Close connection before exiting
        light_utils.mode_duty(UVC_LIGHT_PIN, PWM_PIN_MAP[UVC_LIGHT_PIN][0], 0)
        light_utils.mode_off(UVC_LIGHT_PIN, PWM_PIN_MAP[UVC_LIGHT_PIN][0])