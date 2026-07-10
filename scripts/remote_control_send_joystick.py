import remote_control_utils as rc_utils
import time

# Functions parameters
INPUT_PORT=None
INPUT_BAUDRATE=None
SEND_PORT=None  # Serial port to be selected by the user
SEND_BAUDRATE=None  # Baud rate for the serial communication
END_CHAR='\n'  # End character for the command

# Main Function
if __name__ == "__main__":
    # Input serial port and baud rate for receiving data
    print("-------------Receive Serial Port START-----------------\n")
    try:
        INPUT_BAUDRATE = int(input("Enter baud rate for receiving data (default 115200): ") or 115200)
    except ValueError:
        print("Invalid baud rate. Using default 115200.")
        INPUT_BAUDRATE = 115200
    
    # Select the serial port for receiving data
    INPUT_PORT = rc_utils.select_serial_port(INPUT_PORT)
    print(f"Selected port for receiving data: {INPUT_PORT}")

    if INPUT_PORT is None:
        print("No port selected. Exiting.")
        raise SystemExit(1)

    input_ser = rc_utils.init_serial_connection(INPUT_PORT, INPUT_BAUDRATE)
    if input_ser is None:
        raise SystemExit(1)
    print("-------------Receive Serial Port END-----------------\n")

    # Send serial port and baud rate selection
    print("-------------Send Serial Port START-----------------\n")
    try:
        SEND_BAUDRATE = int(input("Enter baud rate (default 4800): ") or 4800)
    except ValueError:
        print("Invalid baud rate. Using default 4800.")
        SEND_BAUDRATE = 4800

    # Select the serial port
    SEND_PORT = rc_utils.select_serial_port(SEND_PORT)
    print(f"Selected port: {SEND_PORT}")

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
        send_ser.close() # Close connection before exiting
        raise SystemExit(1)
    print("-------------Send Serial Port END-----------------\n")

    try:
        while True:
            # Read data from the input serial port
            if input_ser.in_waiting > 0:
                received_data = input_ser.read(22)
                print(f"Received data: {received_data.hex()}\n-EOF")

                # map 00 - 66 to 0 - 255
                map_val = lambda x: int(max(0, min(255, (128/49) * x + 127 - (128/49) * 53)))

                # Parse joystick input - CONVERT BYTES TO INT
                left_joystick_x = received_data[2]
                left_joystick_y = received_data[4]
                right_joystick_x = received_data[8]
                right_joystick_y = received_data[6]
                
                # Map to 0-255
                mapped_left_x = map_val(left_joystick_x)
                mapped_left_y = map_val(left_joystick_y)
                mapped_right_x = map_val(right_joystick_x)
                mapped_right_y = map_val(right_joystick_y)
                print(f"Left Joystick X raw: {left_joystick_x} ({left_joystick_x:02x}h) → mapped: {mapped_left_x}")
                print(f"Left Joystick Y raw: {left_joystick_y} ({left_joystick_y:02x}h) → mapped: {mapped_left_y}")
                print(f"Right Joystick X raw: {right_joystick_x} ({right_joystick_x:02x}h) → mapped: {mapped_right_x}")
                print(f"Right Joystick Y raw: {right_joystick_y} ({right_joystick_y:02x}h) → mapped: {mapped_right_y}")

                # # Send data
                byte_data = bytes([mapped_left_x, mapped_left_y, mapped_right_x, mapped_right_y])
                response = rc_utils.send_bytes(send_ser, byte_data, wait_time=0.3)
                # print(f"Response: {response}\n-EOF")

                # response = send_ser.read(send_ser.in_waiting)
                # print(response)
                
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        # End
        input_ser.close()
        send_ser.close()

#     # Command Logic
#     try:
#         while True:
#             choice = input('''Select an option:
# 0: exit program
# 1: Send a message
# 2: Send an AT command
# 3: Send bytes
# 4: Motor control (percentage)
# Enter your choice: ''').strip()
#             if choice == "0":
#                 print("Exiting program.")
#                 break
#             elif choice == "1":
#                 msg = input("Enter the message to send: ")
#                 response = rc_utils.send_msg(ser, msg)
#                 print(f"Response: {response}\n-EOF")
#             elif choice == "2":
#                 at_command = input("Enter the AT command to send: ")
#                 response = rc_utils.send_at_command(ser, at_command)
#                 print(f"Response: {response}\n-EOF")
#             elif choice == "3":
#                 # Input 010203, then send 0x01, 0x02, 0x03 as bytes
#                 byte_input = input("Enter the bytes to send (as hex, e.g., '010203'): ")
#                 byte_data = bytes.fromhex(byte_input)
#                 response = rc_utils.send_bytes(ser, byte_data)
#                 print(f"Response: {response}\n-EOF")
#             elif choice == "4":
#                 # Input percentage for left and right motors
#                 try:
#                     left_percentage = int(input("Enter left motor percentage (-100 to 100): "))
#                     right_percentage = int(input("Enter right motor percentage (-100 to 100): "))
#                     if not (-100 <= left_percentage <= 100) or not (-100 <= right_percentage <= 100):
#                         raise ValueError("Percentage must be between -100 and 100.")

#                     # Convert percentage to 0-255, 0-126 is reverse, 127 is stop, 128-255 is forward
#                     left_pwm = 127 + (left_percentage / 100) * 128
#                     right_pwm = 127 + (right_percentage / 100) * 128

#                     # Send bytes
#                     byte_data = bytes([int(left_pwm), int(right_pwm)])
#                     response = rc_utils.send_bytes(ser, byte_data)
#                     print(f"Response: {response}\n-EOF")
#                 except ValueError as ve:
#                     print(f"Invalid input: {ve}")
#                     continue
#             else:
#                 print("Invalid choice. Exiting.")
#     finally:
#         send_ser.close()