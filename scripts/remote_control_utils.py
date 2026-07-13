import serial
import time
import os
import serial.tools.list_ports

def select_baudrate(baudrate=115200):
    try:
        BAUDRATE = int(input(f"Enter baud rate for receiving data (default {baudrate}): ") or baudrate)
    except ValueError:
        print("Invalid baud rate. Using default 115200.")
        BAUDRATE = baudrate
    finally:
        return BAUDRATE

def select_serial_port(port=None):
    """
    List available serial ports and allow user to select one.
    Returns the selected port device string or None if canceled.
    """
    while True:
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("No serial ports found.")
            return None

        print("Available serial ports:")
        for i, port_info in enumerate(ports):
            print(f"{i + 1}: {port_info.device} - {port_info.description}")

        try:
            choice = int(input("Select a port by number (or 0 to exit, r to retry): "))
            if choice == 0:
                return None
            elif 1 <= choice <= len(ports):
                return ports[choice - 1].device
            else:
                print("Invalid selection. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")

def init_serial_connection(port, baudrate):
    """
    Initialize serial connection with given port and baudrate
    """
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2
        )
        return ser
    except Exception as e:
        print(f"Failed to initialize serial connection: {e}")
        return None

def send_config_command(ser, command=None, wait_time=1, end_char='\n'):
    #AT+RF_CONFIG =
    #   <Preamble>,<BW>,<CodeRate>,<SF>,<HopPeriod>,
    #    <Channel>,<Power>
    # Hardcoded first
    # AT+rf_config=16,0,4,12,0,0,4
    config_command = "AT+rf_config=16,1,4,7,0,0,4"
    
    try:
        print(f"Sending command: {command}")
        ser.write((config_command + end_char).encode())
        time.sleep(wait_time)

        response = ser.read(ser.in_waiting)
        return response.decode('utf-8', errors='ignore')
    except Exception as e:
        return f"Error: {e}"

def send_at_command(ser, command, wait_time=1, end_char='\n'):
    """
    Send an AT command over an active serial connection and return the response.
    """
    try:
        ser.write((command + end_char).encode())
        time.sleep(wait_time)

        response = ser.read(ser.in_waiting)
        return response.decode('utf-8', errors='ignore')

    except Exception as e:
        return f"Error: {e}"

def send_msg(ser, msg, wait_time=1, end_char='\n'):
    """
    Send a message over an active serial connection and return the response.
    AT+RF_SEND=<Cnts>,<Interval>, <Len> 
    """
    CNTS = 1
    INTERVAL = 0
    LEN = len(msg)

    try:
        # Send the rf_send command first
        rf_send_command = f"AT+rf_send={CNTS},{INTERVAL},{LEN}"
        print(f"Sending command: {rf_send_command}")
        ser.write((rf_send_command + end_char).encode())
        time.sleep(0.1)

        # Send message
        ser.write((msg).encode())
        time.sleep(wait_time)   # Wait for response

        response = ser.read(ser.in_waiting)
        return response.decode('utf-8', errors='ignore')
    except Exception as e:
        return f"Error: {e}"


def send_bytes(ser, byte_data, wait_time=1, end_char='\n', read_response=True, debug=False):
    """
    Send raw bytes over an active serial connection and return the response.
    """
    CNTS = 1
    INTERVAL = 0
    LEN = len(byte_data)

    try:
        rf_send_command = f"AT+rf_send={CNTS},{INTERVAL},{LEN}"
        
        if debug:
            print(f"Sending command: {rf_send_command}")

        ser.write((rf_send_command + end_char).encode())
        time.sleep(0.1)

        ser.write(byte_data)

        if read_response:
            response = ser.read(ser.in_waiting)
            return response.decode('utf-8', errors='ignore')
        else:
            return "OK"
    except Exception as e:
        return f"Error: {e}"