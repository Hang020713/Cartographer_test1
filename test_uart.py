import serial
import time

# Configure serial port
ser = serial.Serial(
    port='/dev/tty.usbmodem9',  # or /dev/ttyAMA0 for built-in UART
    baudrate=4800,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=2  # 2 second timeout for reading
)

def send_at_command(command, wait_time=1):
    """
    Send AT command and return response
    """
    try:
        # Send command with CR+LF (typical for AT commands)
        ser.write((command + '\r\n').encode())
        time.sleep(wait_time)  # Wait for response
        
        # Read all available data
        response = ser.read(ser.in_waiting)
        return response.decode('utf-8', errors='ignore')
    
    except Exception as e:
        return f"Error: {e}"

# Example usage
if __name__ == "__main__":
    # Test with basic AT command
    response = send_at_command('AT')
    print(f"Response: {response}")
    
    # Check signal quality
    response = send_at_command('AT+CSQ')
    print(f"Signal Quality: {response}")
    
    # Check network registration
    response = send_at_command('AT+CREG?')
    print(f"Network Registration: {response}")
    
    # Close serial port
    ser.close()