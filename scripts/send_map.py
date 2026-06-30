import serial
import sys
import time

# This code is to send the pgm file data with binary format over UART on Win/Mac
# Keyboard interrupt (Ctrl+C) to stop the program
# Usage: python3 send_map.py <pgm_file_path> <serial_port> <baudrate>
# Example: python3 send_map.py /path/to/map.pgm /dev/tty.usbserial-XXXX 115200

# Parameters with default values
FILE_PATH = "/Users/b/Downloads/square_map.pgm"
SERIAL_PORT = "/dev/tty.usbserial-140"
BAUDRATE = 115200

# Open the serial port
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

# Read the PGM file in binary mode
with open(FILE_PATH, 'rb') as f:
    pgm_data = f.read()

try:
    while True:
        # Send the PGM data over UART
        ser.write(pgm_data)
        print(f"Sent {len(pgm_data)} bytes of PGM data over UART.")
        
        # Wait for a short period before sending again
        time.sleep(1)  # Adjust the sleep time as needed

except KeyboardInterrupt:
    print("Program stopped by user.")
    ser.close()