import remote_control_utils as rc_utils

# Functions parameters
PORT=None  # Serial port to be selected by the user
BAUDRATE=None  # Baud rate for the serial communication
END_CHAR='\n'  # End character for the command

# Main Function
if __name__ == "__main__":
    # Select baud rate, default to 4800 if not provided
    try:
        BAUDRATE = int(input("Enter baud rate (default 4800): ") or 4800)
    except ValueError:
        print("Invalid baud rate. Using default 4800.")
        BAUDRATE = 4800

    # Select the serial port
    PORT = rc_utils.select_serial_port(PORT)
    print(f"Selected port: {PORT}")

    if PORT is None:
        print("No port selected. Exiting.")
        raise SystemExit(1)

    # Init serial connection
    ser = rc_utils.init_serial_connection(PORT, BAUDRATE)
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

    # Command Logic
    try:
        while True:
            choice = input('''Select an option:
0: exit program
1: Send a message
2: Send an AT command
3: Send bytes
4: Motor control (percentage)
Enter your choice: ''').strip()
            if choice == "0":
                print("Exiting program.")
                break
            elif choice == "1":
                msg = input("Enter the message to send: ")
                response = rc_utils.send_msg(ser, msg)
                print(f"Response: {response}\n-EOF")
            elif choice == "2":
                at_command = input("Enter the AT command to send: ")
                response = rc_utils.send_at_command(ser, at_command)
                print(f"Response: {response}\n-EOF")
            elif choice == "3":
                # Input 010203, then send 0x01, 0x02, 0x03 as bytes
                byte_input = input("Enter the bytes to send (as hex, e.g., '010203'): ")
                byte_data = bytes.fromhex(byte_input)
                response = rc_utils.send_bytes(ser, byte_data)
                print(f"Response: {response}\n-EOF")
            elif choice == "4":
                # Input percentage for left and right motors
                try:
                    left_percentage = int(input("Enter left motor percentage (-100 to 100): "))
                    right_percentage = int(input("Enter right motor percentage (-100 to 100): "))
                    if not (-100 <= left_percentage <= 100) or not (-100 <= right_percentage <= 100):
                        raise ValueError("Percentage must be between -100 and 100.")

                    # Convert percentage to 0-255, 0-126 is reverse, 127 is stop, 128-255 is forward
                    left_pwm = 127 + (left_percentage / 100) * 128
                    right_pwm = 127 + (right_percentage / 100) * 128

                    # Send bytes
                    byte_data = bytes([int(left_pwm), int(right_pwm)])
                    response = rc_utils.send_bytes(ser, byte_data)
                    print(f"Response: {response}\n-EOF")
                except ValueError as ve:
                    print(f"Invalid input: {ve}")
                    continue
            else:
                print("Invalid choice. Exiting.")
    finally:
        ser.close()