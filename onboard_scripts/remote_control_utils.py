import serial
import time
import os
import serial.tools.list_ports
from enum import IntEnum

INQUERY_PAYLOAD_LEN = 10
STATUS_PAYLOAD_LEN = 16
# STATUS_PAYLOAD_LEN = 4 

class COMMANDS(IntEnum):
    ERROR = -1
    REQUEST_STATUS = 0
    REQUEST_MAP = 1
    SET_MODE = 2
    MANUAL_CONTROL = 3

def get_command_type(command_type):
    try:
        return COMMANDS(int.from_bytes(command_type, byteorder='big'))
    except ValueError:
        return COMMANDS.ERROR

class MODES(IntEnum):
    ERROR = -1
    IDLE = 1
    PRE_WASH = 2
    AUTO = 3
    MANUAL = 4

def get_mode(mode):
    try:
        return MODES(int.from_bytes(mode, byteorder='big'))
    except ValueError:
        return MODES.ERROR

class MODE_STATUS(IntEnum):
    SUCCESS = 0
    FAIL = 1
    ONGOING = 2

def get_mode_status(mode_status):
    try:
        return MODE_STATUS(int.from_bytes(mode_status, byteorder='big'))
    except ValueError:
        return "N/A"

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

def read_frame(ser, start_byte, payload_length):
    # Look for start byte (Message ID byte)
    while True:
        b = ser.read(1)
        if not b:                     # timeout, nothing available
            return None
        if b[0] == start_byte:
            break

    # Read the fixed-length payload
    frame = ser.read(payload_length - 1)
    # print(frame)
    if len(frame) < (payload_length - 1):        # incomplete -> resync next loop
        return None

    return frame

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
    entm_command = "AT+ENTM"
    
    try:
        print(f"Sending command: {config_command}")
        ser.write((config_command + end_char).encode())
        time.sleep(wait_time)
        response = ser.read(ser.in_waiting)
        print(response)

        print(f"Sending command: {entm_command}")
        ser.write((entm_command + end_char).encode())
        time.sleep(wait_time)
        response = ser.read(ser.in_waiting)
        print(response)
        
        # return response.decode('utf-8', errors='ignore')
        # TODO: fix this
        return "OK"
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
        # rf_send_command = f"AT+rf_send={CNTS},{INTERVAL},{LEN}"
        
        # if debug:
        #     print(f"Sending command: {rf_send_command}")

        # print(f"[{time.time()}]: Start writing AT_SEND:1,0,4")
        # ser.write((rf_send_command + end_char).encode())
        # print(f"[{time.time()}]: Start sleep")
        # time.sleep(0.05)    # 10 ms
        # print(f"[{time.time()}]: Done sleep")
        
        # print(f"[{time.time()}]: Start writing data")
        ser.write(byte_data)
        # print(f"[{time.time()}]: done writing data")
        # print(f"[{time.time()}]: Start sleep")
        time.sleep(0.02)    # 20ms
        # print(f"[{time.time()}]: done sleeping")


        if read_response:
            response = ser.read(ser.in_waiting)
            return response.decode('utf-8', errors='ignore')
        else:
            return "OK"
    except Exception as e:
        return f"Error: {e}"