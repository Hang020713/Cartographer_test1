import remote_control_utils as rc_utils
import time
import threading

# Functions parameters
INPUT_PORT=None
INPUT_BAUDRATE=None
SEND_PORT=None  # Serial port to be selected by the user
SEND_BAUDRATE=None  # Baud rate for the serial communication
END_CHAR='\n'  # End character for the command

# Manual Command parameters
mapped_left_x = 127
mapped_left_y = 127
mapped_right_x = 127
mapped_right_y = 127

# Threads
program_stop_event = threading.Event()
send_stop_event = threading.Event()
joystick_lock = threading.Lock()
read_thread = None
send_thread = None

def read_joystick_thread():
    while not program_stop_event.is_set():
        read_joystick()
        # time.sleep(0.01)

def send_manual_command_thread():
    while not program_stop_event.is_set() and not send_stop_event.is_set():
        send_manual_command()
        time.sleep(0.1)

# Joystick functions
def map_joystick_value(x):
    return int(max(0, min(255, (128 / 49) * x + 127 - (128 / 49) * 53)))

def read_joystick():
    global mapped_left_x, mapped_left_y, mapped_right_x, mapped_right_y

    if input_ser.in_waiting > 0:
        received_data = input_ser.read(22)
        # print(f"Received data: {received_data.hex()}\n-EOF")

        # Parse joystick input - CONVERT BYTES TO INT
        left_joystick_x = received_data[2]
        left_joystick_y = received_data[4]
        right_joystick_x = received_data[8]
        right_joystick_y = received_data[6]

        # Map to 0-255
        with joystick_lock:
            mapped_left_x = map_joystick_value(left_joystick_x)
            mapped_left_y = map_joystick_value(left_joystick_y)
            mapped_right_x = map_joystick_value(right_joystick_x)
            mapped_right_y = map_joystick_value(right_joystick_y)

        # print(f"Left Joystick X raw: {left_joystick_x} ({left_joystick_x:02x}h) → mapped: {mapped_left_x}")
        # print(f"Left Joystick Y raw: {left_joystick_y} ({left_joystick_y:02x}h) → mapped: {mapped_left_y}")
        # print(f"Right Joystick X raw: {right_joystick_x} ({right_joystick_x:02x}h) → mapped: {mapped_right_x}")
        # print(f"Right Joystick Y raw: {right_joystick_y} ({right_joystick_y:02x}h) → mapped: {mapped_right_y}")

def send_manual_command():
    global mapped_left_x, mapped_left_y, mapped_right_x, mapped_right_y

    with joystick_lock:
        left_x = mapped_left_x
        left_y = mapped_left_y
        right_x = mapped_right_x
        right_y = mapped_right_y

    byte_data = bytes([left_x, left_y, right_x, right_y])
    response = rc_utils.send_bytes(send_ser, byte_data, wait_time=0.1, read_response=False)
    # print(f"Response: {response}\n-EOF")

# Main Function
if __name__ == "__main__":
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
        send_ser.close() # Close connection before exiting
        raise SystemExit(1)
    print("-------------Send Serial Port END-----------------\n")

    program_stop_event.clear()
    send_stop_event.set()
    if read_thread is None or not read_thread.is_alive():
        read_thread = threading.Thread(target=read_joystick_thread, daemon=True)
        read_thread.start()
    print("Joystick reader thread started and will keep running.")

    # Command Logic
    try:
        while True:
            choice = input('''Select an option:
0: exit program
1: Set Mode
2: Stop manual sending (reader stays active)
Enter your choice: ''').strip()
            if choice == "0":
                print("Exiting program.")
                break
            elif choice == "1":
                print("TESTING NOT AVAILABLE")
            elif choice == "2":
                if send_thread is not None and send_thread.is_alive():
                    print("Manual sending is already running. Press Ctrl+C to stop it.")
                else:
                    print("Starting manual sending. Press Ctrl+C to stop and return to the menu.")
                    send_stop_event.clear()
                    send_thread = threading.Thread(target=send_manual_command_thread, daemon=True)
                    send_thread.start()

                    try:
                        while True:
                            time.sleep(0.5)
                    except KeyboardInterrupt:
                        print("Stopping manual sending. Returning to the menu.")
                        send_stop_event.set()
                        if send_thread is not None:
                            send_thread.join(timeout=1)
                            send_thread = None
            else:
                print("Invalid choice. Exiting.")
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        program_stop_event.set()
        send_stop_event.set()
        if read_thread is not None:
            read_thread.join(timeout=1)
        if send_thread is not None:
            send_thread.join(timeout=1)
        input_ser.close()
        send_ser.close()