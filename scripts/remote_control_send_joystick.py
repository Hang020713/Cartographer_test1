import remote_control_utils as rc_utils
import time
import threading
from enum import IntEnum

# Debug parameter
HAVE_JOYSTICK=False
DEBUG_JOYSTICK=False

# Functions parameters
INPUT_PORT=None
INPUT_BAUDRATE=None
SEND_PORT=None  # Serial port to be selected by the user
SEND_BAUDRATE=None  # Baud rate for the serial communication
END_CHAR='\n'  # End character for the command

# Joystick parameters
JOYSTICK_BIT_LEN = 22
LX_BIT = 2
LY_BIT = 4
RX_BIT = 8
RY_BIT = 6
BRUSH_DIR_BIT = 18
BRUSH_SPEED_BIT = 10
LIGHT_BIT = 12

# Payload parameter
MESSAGE_ID = 0xAA
ID = 0x00

# Manual Command parameters
mapped_left_x = 127
mapped_left_y = 127
mapped_right_x = 127
mapped_right_y = 127
mapped_brush_dir = 1   # 0: idle, 1: rotate up, 2: rotate down
mapped_brush_speed = 100 # 0 - 100
mapped_light_pct = 100

# Threads
program_stop_event = threading.Event()
joystick_lock = threading.Lock()
read_joystick_thread = None
receive_lora_thread = None

# Lora received parameters


def read_joystick_thread_func():
    while not program_stop_event.is_set():
        read_joystick()
        time.sleep(0.01)

def receive_lora_thread_func():
    while not program_stop_event.is_set():
        receive_lora_response()
        time.sleep(0.01)

def receive_lora_response():
    # Wait for status response
    received_data = rc_utils.read_frame(send_ser, MESSAGE_ID, rc_utils.STATUS_PAYLOAD_LEN)
    if received_data is None:
        return
    print(received_data)

    mode = received_data[1:2]
    mode_status = received_data[2:3]
    print(f"[{time.time()}] mode: {rc_utils.get_mode(mode).name}")
    print(f"[{time.time()}] mode_status: {rc_utils.get_mode_status(mode_status).name}")

# Joystick functions
def map_joystick_value(x):
    return int(max(0, min(255, (128 / 49) * x + 127 - (128 / 49) * 53)))

def read_joystick():
    global mapped_left_x, mapped_left_y, mapped_right_x, mapped_right_y, mapped_brush_dir, mapped_brush_speed, mapped_light_pct

    if input_ser.in_waiting > 0:
        received_data = input_ser.read(JOYSTICK_BIT_LEN)
        # print(f"Received data: {received_data.hex()}\n-EOF")

        # Parse joystick input - CONVERT BYTES TO INT
        left_joystick_x = received_data[LX_BIT]
        left_joystick_y = received_data[LY_BIT]
        right_joystick_x = received_data[RX_BIT]
        right_joystick_y = received_data[RY_BIT]
        brush_dir = received_data[BRUSH_DIR_BIT]
        brush_speed = received_data[BRUSH_SPEED_BIT]
        light_pct = received_data[LIGHT_BIT]

        # Map to 0-255
        with joystick_lock:
            # joystick
            mapped_left_x = map_joystick_value(left_joystick_x)
            mapped_left_y = map_joystick_value(left_joystick_y)
            mapped_right_x = map_joystick_value(right_joystick_x)
            mapped_right_y = map_joystick_value(right_joystick_y)

            # brush
            mapped_brush_dir = brush_dir - 128
            mapped_brush_speed = 100 if brush_speed > 100 else brush_speed
            mapped_light_pct = 100 if light_pct > 100 else light_pct

        # print(f"Left Joystick X raw: {left_joystick_x} ({left_joystick_x:02x}h) → mapped: {mapped_left_x}")
        # print(f"Left Joystick Y raw: {left_joystick_y} ({left_joystick_y:02x}h) → mapped: {mapped_left_y}")
        # print(f"Right Joystick X raw: {right_joystick_x} ({right_joystick_x:02x}h) → mapped: {mapped_right_x}")
        # print(f"Right Joystick Y raw: {right_joystick_y} ({right_joystick_y:02x}h) → mapped: {mapped_right_y}")
        if DEBUG_JOYSTICK:
            print(f"[{time.time()}]LX: {left_joystick_x}, LY: {left_joystick_y}, RX: {right_joystick_x}, RY: {right_joystick_y}")
            print(f"[{time.time()}]Brush Dir: {mapped_brush_dir}({brush_dir}), speed: {mapped_brush_speed}({brush_speed})")
            print(f"[{time.time()}]Light: {mapped_light_pct}({light_pct})")

def send_manual_control(read_response=False):
    global mapped_left_x, mapped_left_y, mapped_right_x, mapped_right_y, mapped_brush_dir, mapped_brush_speed, mapped_light_pct

    # LX, LY, RX, RY, Brush dir, Brush speed, light
    byte_data = bytes([MESSAGE_ID, ID, rc_utils.COMMANDS.MANUAL_CONTROL, 
                       mapped_left_x, mapped_left_y, mapped_right_x, mapped_right_y,
                       mapped_brush_dir, mapped_brush_speed, mapped_light_pct
                    ])
    response = rc_utils.send_bytes(send_ser, byte_data, wait_time=0.3, read_response=read_response)
    return response

def send_request_status(read_response=False):
    byte_data = bytes([MESSAGE_ID, ID, rc_utils.COMMANDS.REQUEST_STATUS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    response = rc_utils.send_bytes(send_ser, byte_data, wait_time=0.3, read_response=read_response)
    return response

# Main Function
if __name__ == "__main__":
    input_ser = None
    if HAVE_JOYSTICK:
        # Input serial port and baud rate for receiving data
        print("-------------Receive Serial Port START-----------------\n")
        INPUT_BAUDRATE = rc_utils.select_baudrate(115200)
        print(f"Selected baudrate: {INPUT_BAUDRATE}")
        
        # Select the serial port for receiving data
        INPUT_PORT = rc_utils.select_serial_port(INPUT_PORT)
        print(f"Selected port for receiving data: {INPUT_PORT}")

        # Check port selected
        if INPUT_PORT is None:
            print("No port selected. Exiting.")
            raise SystemExit(1)

        # Init serial connection
        input_ser = rc_utils.init_serial_connection(INPUT_PORT, INPUT_BAUDRATE)
        if input_ser is None:
            raise SystemExit(1)
        print("-------------Receive Serial Port END-----------------\n")

    # Send serial port and baud rate selection
    print("-------------Send Serial Port START-----------------\n")
    SEND_BAUDRATE = rc_utils.select_baudrate(4800)
    print(f"Select baudrate: {SEND_BAUDRATE}")

    # Select the serial port
    SEND_PORT = rc_utils.select_serial_port(SEND_PORT)
    print(f"Selected port: {SEND_PORT}")

    # Check port selected
    if SEND_PORT is None:
        print("No port selected. Exiting.")
        raise SystemExit(1)

    # Init serial connection
    send_ser = rc_utils.init_serial_connection(SEND_PORT, SEND_BAUDRATE)
    if send_ser is None:
        raise SystemExit(1)

    # Configure the device
    response = rc_utils.send_config_command(send_ser, end_char=END_CHAR)
    print(f"Response: {response}\n-EOF")
    if "OK" in response:
        print("Configuration command sent successfully.")
    else:
        print("Configuration command failed or returned unexpected response.")
        input_ser.close()
        send_ser.close()
        raise SystemExit(1)
    print("-------------Send Serial Port END-----------------\n")

    # Init threads
    program_stop_event.clear()
    if receive_lora_thread is None or not receive_lora_thread.is_alive():
        receive_lora_thread = threading.Thread(target=receive_lora_thread_func, daemon=True)
        receive_lora_thread.start()
    print("lora receive thread started and will keep running.")

    if HAVE_JOYSTICK:
        if read_joystick_thread is None or not read_joystick_thread.is_alive():
            read_joystick_thread = threading.Thread(target=read_joystick_thread_func, daemon=True)
            read_joystick_thread.start()
        print("Joystick reader thread started and will keep running.")

    # Command Logic
    try:
        while True:
            choice = input('''Select an option:
0: exit program
1: Set Mode
2: Keep sending manual command
3: Request status
Enter your choice: ''').strip()
            if choice == "0":
                print("Exiting program.")
                break
            elif choice == "1":
                print("TESTING NOT AVAILABLE")
            elif choice == "2":
                print("Starting manual sending. Press Ctrl+C to stop and return to the menu.")

                flag = True
                while True:
                    try:
                        # Send manual command
                        send_manual_control()
                        time.sleep(0.2)

                        # Ask for status
                        send_request_status()
                        time.sleep(0.2)
                        # print("request done")

                        # Program end
                        if not flag:
                            break
                    except KeyboardInterrupt:
                        print("Stopping manual sending. Returning to the menu.")
                        flag = False

            elif choice == "3":
                byte_data = bytes([MESSAGE_ID, ID, rc_utils.COMMANDS.REQUEST_STATUS, 0x00, 0x00, 0x00, 0x00])
                response = rc_utils.send_bytes(send_ser, byte_data, read_response=False)
                # print(f"Response: {response}\n-EOF")
            else:
                print("Invalid choice. Exiting.")
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        program_stop_event.set()
        if receive_lora_thread is not None:
            receive_lora_thread.join(timeout=1)
        if read_joystick_thread is not None:
            read_joystick_thread.join(timeout=1)
        if not input_ser == None: 
            input_ser.close()
        send_ser.close()