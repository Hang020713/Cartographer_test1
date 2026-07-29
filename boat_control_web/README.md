# Boat Control UI

A Streamlit-based user interface for remote control of ArduPilot-powered unmanned boats using MAVLink communication.

## Project Structure

```
boat_control_ui/
├── app.py                 # Main Streamlit application
├── config.py              # Configuration constants and settings
├── mavlink_controller.py  # MAVLink communication controller
└── bms_controller.py      # Battery management communication controller
```

## Features

- Real-time connection status monitoring
- Manual and Guided flight mode control
- Throttle and steering control
- Arm/disarm functionality
- Live telemetry display
- Logging system for debugging
- Battery management system (BMS) communication

## Prerequisites

- Python 3.8+
- ArduPilot-compatible hardware (e.g., Pixhawk 6c)
- Serial connection to the vehicle

## Installation

1. Install required Python packages:

```bash
pip install streamlit pymavlink
```
or
```bash
cd ~/boat_control_web
pip3 install -r requirements.txt
```

2. Configure serial port settings in `config.py`:

```python
SERIAL_PORT = "COM12"  # Change to your serial port (e.g., "/dev/ttyUSB0" on Linux)
SERIAL_BAUD = 115200
```

## Usage

1. Ensure your ArduPilot vehicle is powered on and connected via serial.

2. Run the Streamlit application:

```bash
streamlit run app.py
```

3. Open the provided URL in your web browser.

4. Connect to the vehicle using the interface.

5. Arm the vehicle and use the controls for throttle and steering.

## Configuration

Edit `config.py` to customize:

- Serial port and baud rate
- Flight modes
- Control constants (throttle and steering ranges)
- UI settings (page title, layout, etc.)

## Dependencies

- `streamlit`: Web-based UI framework
- `pymavlink`: MAVLink protocol implementation for Python

## Troubleshooting

- **Connection Issues**: Verify serial port settings and ensure the vehicle is powered on.
- **Permission Errors**: On Linux, you may need to add your user to the `dialout` group: `sudo usermod -a -G dialout $USER`
- **Import Errors**: Ensure all dependencies are installed.

## License

MIT License 