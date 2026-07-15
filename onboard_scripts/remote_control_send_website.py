import remote_control_utils as rc_utils
import time
import threading
import copy
from enum import IntEnum
from flask import Flask, jsonify, render_template_string

# Debug parameter
HAVE_JOYSTICK=False
DEBUG_JOYSTICK=False
WEB_DASHBOARD=True          # NEW: enable the web dashboard
WEB_HOST="0.0.0.0"          # NEW
WEB_PORT=5050               # NEW

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
mapped_light_pct = 50

# Threads
program_stop_event = threading.Event()
joystick_lock = threading.Lock()
read_joystick_thread = None
receive_lora_thread = None
web_thread = None                       # NEW

# Lora received parameters
CURRENT_LSB = 0.001
kTempOffset = -45.0
kTempScale = 175.0
kHumScale = 100.0
kRawMax = 65535.0

# ---------------------------------------------------------------------------
# NEW: Shared status store for the web dashboard
# ---------------------------------------------------------------------------
status_lock = threading.Lock()
latest_status = {
    "timestamp": None,
    "mode": None,
    "mode_status": None,
    "sensor_channels": [None, None, None, None],  # 4 motor currents (A)
    "humidity_pct": None,
    "temperature_c": None,
    "humidity_raw": None,
    "temperature_raw": None,
}


def update_latest_status(parsed):
    """Copy parsed status into the shared store for the dashboard."""
    mode = parsed.get("mode")
    mode_status = parsed.get("mode_status")
    with status_lock:
        latest_status["timestamp"] = time.time()
        latest_status["mode"] = mode.name if hasattr(mode, "name") else mode
        latest_status["mode_status"] = (
            mode_status.name if hasattr(mode_status, "name") else mode_status
        )
        latest_status["sensor_channels"] = parsed.get("sensor_channels",
                                                       [None, None, None, None])
        latest_status["humidity_pct"] = parsed.get("humidity_pct")
        latest_status["temperature_c"] = parsed.get("temperature_c")
        latest_status["humidity_raw"] = parsed.get("humidity_raw")
        latest_status["temperature_raw"] = parsed.get("temperature_raw")


# ---------------------------------------------------------------------------
# NEW: Flask web dashboard
# ---------------------------------------------------------------------------
flask_app = Flask(__name__)

DASHBOARD_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Dashboard</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, sans-serif;
            background: #0f172a; color: #e2e8f0; min-height: 100vh; padding: 16px;
        }
        h1.page-title {
            text-align: center; font-size: 1.4rem; margin-bottom: 6px;
            color: #38bdf8; letter-spacing: 1px;
        }
        .status-line {
            text-align: center; font-size: 0.8rem; color: #64748b; margin-bottom: 16px;
        }
        .status-line .dot {
            display: inline-block; width: 8px; height: 8px; border-radius: 50%;
            background: #ef4444; margin-right: 6px; vertical-align: middle;
        }
        .status-line .dot.live { background: #22c55e; }
        .dashboard {
            display: grid; grid-template-columns: 1fr 2fr 1fr; gap: 16px; align-items: start;
        }
        .panel {
            background: #1e293b; border: 1px solid #334155; border-radius: 12px; padding: 16px;
        }
        .panel h2 {
            font-size: 1.05rem; margin-bottom: 14px; color: #7dd3fc;
            border-bottom: 1px solid #334155; padding-bottom: 8px;
        }
        .card {
            background: #0f172a; border-radius: 8px; padding: 10px 12px; margin-bottom: 10px;
            display: flex; justify-content: space-between; align-items: center;
        }
        .card .label { color: #94a3b8; font-size: 0.85rem; }
        .card .value { font-weight: 600; font-size: 1.05rem; color: #f1f5f9; }
        .card .unit  { color: #64748b; font-size: 0.75rem; margin-left: 2px; }
        .section-title {
            font-size: 0.8rem; text-transform: uppercase; letter-spacing: 1px;
            color: #64748b; margin: 12px 0 6px;
        }
        .image-wrap {
            width: 100%; aspect-ratio: 4 / 3; background: #0f172a;
            height: 100%;      /* must have a real height to fill */
            overflow: hidden;
            border: 2px dashed #475569; border-radius: 10px;
            display: flex; align-items: center; justify-content: center; overflow: hidden;
        }
        .image-wrap img {
            height: 100%;
            object-fit: cover;
            transition: transform 0.3s ease;
            
            /* Prevents blurriness during upscale/zoom */
            image-rendering: -moz-crisp-edges; /* Firefox */
            image-rendering: pixelated;        /* Chrome, Edge, Safari */
        }
        .image-placeholder { color: #475569; text-align: center; font-size: 0.95rem; }
        .mode-badges { display: flex; gap: 8px; margin-top: 12px; }
        .badge {
            flex: 1; text-align: center; background: #0f172a; border-radius: 8px;
            padding: 8px; font-size: 0.85rem;
        }
        .badge .k { color: #64748b; font-size: 0.7rem; text-transform: uppercase; }
        .badge .v { color: #38bdf8; font-weight: 600; margin-top: 2px; }
        .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
        @media (max-width: 850px) { .dashboard { grid-template-columns: 1fr; } }
    </style>
</head>
<body>
    <h1 class="page-title">ROBOT MONITORING DASHBOARD</h1>
    <div class="status-line">
        <span class="dot" id="live-dot"></span>
        <span id="last-update">Waiting for data...</span>
    </div>

    <div class="dashboard">

        <!-- LEFT COLUMN -->
        <div class="panel">
            <h2>Motors & Environment</h2>
            <div class="section-title">Motor Currents</div>
            <div class="card"><span class="label">Motor 1</span><span class="value"><span id="m0">--</span><span class="unit">A</span></span></div>
            <div class="card"><span class="label">Motor 2</span><span class="value"><span id="m1">--</span><span class="unit">A</span></span></div>
            <div class="card"><span class="label">Motor 3</span><span class="value"><span id="m2">--</span><span class="unit">A</span></span></div>
            <div class="card"><span class="label">Motor 4</span><span class="value"><span id="m3">--</span><span class="unit">A</span></span></div>

            <div class="section-title">Environment</div>
            <div class="card"><span class="label">Humidity</span><span class="value"><span id="humidity">--</span><span class="unit">%</span></span></div>
            <div class="card"><span class="label">Temperature</span><span class="value"><span id="temperature">--</span><span class="unit">&deg;C</span></span></div>
        </div>

        <!-- CENTER COLUMN -->
        <div class="panel">
            <h2>Camera / Image</h2>
            <div class="image-wrap">
                <img src="/static/pure_square_map.png" alt="map">
            </div>
            <div class="mode-badges">
                <div class="badge"><div class="k">Mode</div><div class="v" id="mode">--</div></div>
                <div class="badge"><div class="k">Mode Status</div><div class="v" id="mode_status">--</div></div>
            </div>
        </div>

        <!-- RIGHT COLUMN -->
        <div class="panel">
            <h2>IMU Data</h2>
            <div class="section-title">Accelerometer (m/s&sup2;)</div>
            <div class="grid-2">
                <div class="card"><span class="label">X</span><span class="value" id="ax">N/A</span></div>
                <div class="card"><span class="label">Y</span><span class="value" id="ay">N/A</span></div>
                <div class="card"><span class="label">Z</span><span class="value" id="az">N/A</span></div>
            </div>
            <div class="section-title">Gyroscope (rad/s)</div>
            <div class="grid-2">
                <div class="card"><span class="label">X</span><span class="value" id="gx">N/A</span></div>
                <div class="card"><span class="label">Y</span><span class="value" id="gy">N/A</span></div>
                <div class="card"><span class="label">Z</span><span class="value" id="gz">N/A</span></div>
            </div>
            <div class="section-title">Orientation (&deg;)</div>
            <div class="card"><span class="label">Roll</span><span class="value" id="roll">N/A</span></div>
            <div class="card"><span class="label">Pitch</span><span class="value" id="pitch">N/A</span></div>
            <div class="card"><span class="label">Yaw</span><span class="value" id="yaw">N/A</span></div>
        </div>

    </div>

    <script>
        function fmt(v, digits) {
            if (v === null || v === undefined) return "--";
            return Number(v).toFixed(digits);
        }
        async function refresh() {
            try {
                const res = await fetch('/api/data');
                const d = await res.json();
                const ch = d.sensor_channels || [];
                for (let i = 0; i < 4; i++) {
                    document.getElementById('m' + i).textContent = fmt(ch[i], 3);
                }
                document.getElementById('humidity').textContent = fmt(d.humidity_pct, 1);
                document.getElementById('temperature').textContent = fmt(d.temperature_c, 1);
                document.getElementById('mode').textContent = d.mode ?? '--';
                document.getElementById('mode_status').textContent = d.mode_status ?? '--';

                const dot = document.getElementById('live-dot');
                const lbl = document.getElementById('last-update');
                if (d.timestamp) {
                    const age = Date.now() / 1000 - d.timestamp;
                    if (age < 5) {
                        dot.classList.add('live');
                        lbl.textContent = 'Live \u2014 updated ' + age.toFixed(1) + 's ago';
                    } else {
                        dot.classList.remove('live');
                        lbl.textContent = 'Stale \u2014 last data ' + age.toFixed(0) + 's ago';
                    }
                } else {
                    dot.classList.remove('live');
                    lbl.textContent = 'Waiting for data...';
                }
            } catch (e) {
                document.getElementById('last-update').textContent = 'Connection error';
            }
        }
        setInterval(refresh, 500);
        refresh();
    </script>
</body>
</html>
"""


@flask_app.route("/")
def dashboard_index():
    return render_template_string(DASHBOARD_HTML)


@flask_app.route("/api/data")
def dashboard_data():
    with status_lock:
        return jsonify(copy.deepcopy(latest_status))


def web_thread_func():
    # Disable the reloader (it would spawn a second process and conflict with threads)
    flask_app.run(host=WEB_HOST, port=WEB_PORT, debug=False,
                  use_reloader=False, threaded=True)


# ---------------------------------------------------------------------------
# Existing parsing / serial logic
# ---------------------------------------------------------------------------
def parse_status_payload(raw_payload):
    if raw_payload is None:
        return None

    payload = bytes(raw_payload)
    print(f"[status] raw bytes: {payload.hex()}")

    if len(payload) < 3:
        print("[status] payload is too short to parse")
        return None

    parsed = {
        "id": payload[0],
        "mode": rc_utils.get_mode(payload[1:2]),
        "mode_status": rc_utils.get_mode_status(payload[2:3]),
    }

    sensor_channels = []
    for channel_index in range(4):
        start = 3 + channel_index * 2
        end = start + 2
        if end <= len(payload):
            sensor_channels.append(int.from_bytes(payload[start:end], byteorder="big") * CURRENT_LSB)
        else:
            sensor_channels.append(None)

    parsed["sensor_channels"] = sensor_channels

    if len(payload) >= 15:
        humidity_raw = int.from_bytes(payload[11:13], byteorder="big")
        temperature_raw = int.from_bytes(payload[13:15], byteorder="big")
        parsed["humidity_raw"] = humidity_raw
        parsed["temperature_raw"] = temperature_raw
        parsed["humidity_pct"] = kHumScale * (humidity_raw / kRawMax)
        parsed["temperature_c"] = kTempOffset + kTempScale * (temperature_raw / kRawMax)

    return parsed


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

    parsed_status = parse_status_payload(received_data)
    if parsed_status is None:
        return

    # NEW: push the latest values to the web dashboard store
    update_latest_status(parsed_status)

    mode_name = parsed_status["mode"].name if hasattr(parsed_status["mode"], "name") else parsed_status["mode"]
    mode_status_name = parsed_status["mode_status"].name if hasattr(parsed_status["mode_status"], "name") else parsed_status["mode_status"]

    print(f"[{time.time()}] mode: {mode_name}")
    print(f"[{time.time()}] mode_status: {mode_status_name}")
    print(f"[{time.time()}] sensor channels: {parsed_status['sensor_channels']}")
    if "humidity_raw" in parsed_status:
        print(f"[{time.time()}] humidity: {parsed_status['humidity_raw']} (raw) -> {parsed_status['humidity_pct']} %")
        print(f"[{time.time()}] temperature: {parsed_status['temperature_raw']} (raw) -> {parsed_status['temperature_c']} °C")

# Joystick functions
def map_joystick_value(x):
    return int(max(0, min(255, (128 / 49) * x + 127 - (128 / 49) * 53)))

def read_frame_2(ser):
    # Look for start byte (Message ID byte)
    while True:
            b = ser.read(2)
            if not b:                     # timeout, nothing available
                return
            if b[0] == 0x0a and b[1] == 0x0d:
                break

    # print(frame)
    frame = ser.read(20)
    if len(frame) < (20):        # incomplete -> resync next loop
        return None

    return frame

def read_joystick():
    global mapped_left_x, mapped_left_y, mapped_right_x, mapped_right_y, mapped_brush_dir, mapped_brush_speed, mapped_light_pct

    if input_ser.in_waiting > 0:
        # received_data = input_ser.read(JOYSTICK_BIT_LEN)
        # Look for start byte (Message ID byte)
        received_data = b"\x0a" + b"\x0d" + read_frame_2(input_ser)
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
        if input_ser is not None:
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

    # NEW: start the web dashboard thread
    if WEB_DASHBOARD:
        if web_thread is None or not web_thread.is_alive():
            web_thread = threading.Thread(target=web_thread_func, daemon=True)
            web_thread.start()
        print(f"Web dashboard started at http://{WEB_HOST}:{WEB_PORT}  "
              f"(open http://localhost:{WEB_PORT} in your browser)")

    # Command Logic
    try:
        while True:
            send_manual_control()
            time.sleep(0.2)

            # Ask for status
            send_request_status()
            time.sleep(0.2)
            # print("request done")
#             choice = input('''Select an option:
# 0: exit program
# 1: Set Mode
# 2: Keep sending manual command
# 3: Request status
# Enter your choice: ''').strip()
#             if choice == "0":
#                 print("Exiting program.")
#                 break
#             elif choice == "1":
#                 print("TESTING NOT AVAILABLE")
#             elif choice == "2":
#                 print("Starting manual sending. Press Ctrl+C to stop and return to the menu.")

#                 flag = True
#                 while True:
#                     try:
#                         # Send manual command
#                         send_manual_control()
#                         time.sleep(0.2)

#                         # Ask for status
#                         send_request_status()
#                         time.sleep(0.2)
#                         # print("request done")

#                         # Program end
#                         if not flag:
#                             break
#                     except KeyboardInterrupt:
#                         print("Stopping manual sending. Returning to the menu.")
#                         flag = False

#             elif choice == "3":
#                 byte_data = bytes([MESSAGE_ID, ID, rc_utils.COMMANDS.REQUEST_STATUS, 
#                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#                                    ])
#                 response = rc_utils.send_bytes(send_ser, byte_data, read_response=False)
#                 # print(f"Response: {response}\n-EOF")
#             else:
#                 print("Invalid choice. Exiting.")
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        program_stop_event.set()
        if receive_lora_thread is not None:
            receive_lora_thread.join(timeout=1)
        if read_joystick_thread is not None:
            read_joystick_thread.join(timeout=1)
        if input_ser is not None:
            input_ser.close()
        send_ser.close()