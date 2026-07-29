import subprocess
import streamlit as st
from mavlink_controller import MavController

from bms_controller import BMSController
from sensor_controller import SensorController
from config import (
    PAGE_TITLE, PAGE_ICON, LAYOUT, SIDEBAR_STATE, MSG_WARN_CONNECTION_REQUIRED, STEERING_CENTER_PWM,
    STEERING_LEFT_PWM, STEERING_RIGHT_PWM,
    SERVO_LEFT_CHANNEL, SERVO_RIGHT_CHANNEL, FLIGHT_MODES, SERIAL_PORT, SERIAL_BAUD,
    BRUSH_LEFT_CHANNEL, BRUSH_RIGHT_CHANNEL, BRUSH_PWM_CENTER,
    BMS_PORT, BMS_BAUD,
    SENSOR_I2C_DEVICE, SENSOR_SHT3X_ADDR, SENSOR_INA4230_ADDRS,
    SENSOR_CURRENT_CHANNELS, SENSOR_POLL_INTERVAL,
    LEVEL_PORT, LEVEL_BAUD, LEVEL_SLAVE_ID,
)

st.set_page_config(
    page_title=PAGE_TITLE,
    page_icon=PAGE_ICON,
    layout=LAYOUT,
    initial_sidebar_state=SIDEBAR_STATE
)

def _get_session_var(name, default=None):
    return st.session_state.get(name, default)

@st.cache_resource
def get_mav_controller():
    print("Initializing MavController...")
    return MavController(port=SERIAL_PORT, baud=SERIAL_BAUD)


@st.cache_resource
def get_bms_controller():
    print("Initializing BMSController...")
    return BMSController(port=BMS_PORT, baudrate=BMS_BAUD)


@st.cache_resource
def get_sensor_controller():
    print("Initializing SensorController...")
    return SensorController(
        i2c_device=SENSOR_I2C_DEVICE,
        sht3x_addr=SENSOR_SHT3X_ADDR,
        ina4230_addrs=SENSOR_INA4230_ADDRS,
        current_channels=SENSOR_CURRENT_CHANNELS,
        level_port=LEVEL_PORT,
        level_baud=LEVEL_BAUD,
        level_slave_id=LEVEL_SLAVE_ID,
        poll_interval=SENSOR_POLL_INTERVAL,
    )

def init_session_state():
    defaults = {
        'is_connected': False,
        'is_armed': False,
        'left_motor': 0,
        'right_motor': 0,
        'movement_mode': 'stopped',
        'flight_mode': 'UNKNOWN',
        'brush_on': False,
        'brush_speed': 50,
        'bms_connected': False,
        'bms_voltage': 0.0,
        'bms_soc': 0.0,
        'bms_charging_current': 0.0,
        'bms_discharging_current': 0.0,
        'bms_capacity': 0.0,
        'bms_temp_sensors': [],
        # Sensor monitor
        'sensor_temperature': None,
        'sensor_humidity': None,
        'sensor_sht3x_ok': False,
        'sensor_currents': {},
        'sensor_ina4230_ok': False,
        'sensor_level_value': None,
        'sensor_level_unit': '',
        'sensor_level_ok': False,
        '_initialized': True
    }

    if not hasattr(st.session_state, '_initialized') or not st.session_state._initialized:
        for key, value in defaults.items():
            if key not in st.session_state:
                st.session_state[key] = value
        st.session_state._initialized = True

    if "ctrl" not in st.session_state:
        st.session_state.ctrl = get_mav_controller()

    if "bms" not in st.session_state:
        st.session_state.bms = get_bms_controller()

    if "sensor" not in st.session_state:
        st.session_state.sensor = get_sensor_controller()
        
def render_header():
    col1, col2, col3 = st.columns([3, 1, 1])
    with col2:
        if _get_session_var('is_connected', False):
            st.write('🟢 Connected')
        else:
            st.write('🔴 Disconnected')

    with col3:
        if _get_session_var('is_armed', False):
            st.write('🔓 Armed')
        else:
            st.write('🔒 Disarmed')

def sync_controller_status():
    try:
        status = st.session_state.ctrl.get_status()
        st.session_state.is_connected = status.get("is_connected", False)
        st.session_state.is_armed = status.get("is_armed", False)
        st.session_state.flight_mode = status.get("flight_mode", "UNKNOWN")
        st.session_state.brush_on = status.get("brush_enabled", False)
        st.session_state.brush_speed = status.get("brush_speed_pct", 0)
    except Exception as e:
        st.error(f"Failed to sync controller status: {e}")
        
def sync_bms_status():
    try:
        status = st.session_state.bms.get_status()
        st.session_state.bms_connected = status.get("bms_connected", False)
        st.session_state.bms_voltage = status.get("voltage", 0.0)
        st.session_state.bms_soc = status.get("soc", 0.0)
        st.session_state.bms_charging_current = status.get("charging_current", 0.0)
        st.session_state.bms_discharging_current = status.get("discharging_current", 0.0)
        st.session_state.bms_capacity = status.get("capacity", 0.0)
        st.session_state.bms_temp_sensors = status.get("temp_sensors", [])
    except Exception as e:
        st.error(f"Failed to sync BMS status: {e}")


def sync_sensor_status():
    try:
        status = st.session_state.sensor.get_status()
        st.session_state.sensor_sht3x_ok = status.get("sht3x_ok", False)
        st.session_state.sensor_temperature = status.get("temperature")
        st.session_state.sensor_humidity = status.get("humidity")
        st.session_state.sensor_ina4230_ok = status.get("ina4230_ok", False)
        st.session_state.sensor_currents = status.get("currents", {})
        st.session_state.sensor_level_ok = status.get("level_ok", False)
        st.session_state.sensor_level_value = status.get("level_value")
        st.session_state.sensor_level_unit = status.get("level_unit", "")
    except Exception as e:
        st.error(f"Failed to sync sensor status: {e}")


def render_system_control():
    st.subheader("⚙️ System Control")

    if "shutdown_confirm" not in st.session_state:
        st.session_state.shutdown_confirm = False

    col1, col2 = st.columns(2)
    with col1:
        if st.button(
            "🔓 Arm",
            use_container_width=True,
            disabled=not _get_session_var('is_connected', False)
                     or _get_session_var('is_armed', False)
        ):
            arm_disarm(True)
            st.rerun()

    with col2:
        if st.button(
            "🔒 Disarm",
            use_container_width=True,
            disabled=not _get_session_var('is_connected', False)
                     or not _get_session_var('is_armed', False)
        ):
            arm_disarm(False)
            st.rerun()

    st.divider()

    if not st.session_state.shutdown_confirm:
        if st.button(
            "🔌 Shutdown Raspberry Pi",
            use_container_width=True,
            type="secondary",
            help="Shutdown Raspberry Pi Ubuntu system (requires confirmation)"
        ):
            st.session_state.shutdown_confirm = True
            st.rerun()
    else:
        st.warning("⚠️ Are you sure you want to shutdown? This will turn off the Raspberry Pi Ubuntu system!")
        col1, col2 = st.columns(2)
        with col1:
            if st.button("✅ Confirm Shutdown", use_container_width=True, type="primary"):
                shutdown_system()
        with col2:
            if st.button("❌ Cancel", use_container_width=True):
                st.session_state.shutdown_confirm = False
                st.rerun()

def arm_disarm(boolean: bool):
    if boolean:
        st.session_state.ctrl.arm()
    else:
        st.session_state.ctrl.disarm()

def shutdown_system():
    try:
        st.session_state.ctrl.disarm()
        st.session_state.ctrl.shutdown()
        st.info("Shutting down Raspberry Pi Ubuntu system...")
        subprocess.run(["sudo", "shutdown", "now"], check=True)
    except Exception as e:
        st.error(f"Failed to shutdown Raspberry Pi Ubuntu system: {e}")

def render_mode_selection():
    """Render flight mode selection"""
    st.subheader("🎮 Mode Selection")

    modes = FLIGHT_MODES
    current_flight_mode = _get_session_var('flight_mode', 'UNKNOWN')
    current_idx = modes.index(current_flight_mode) if current_flight_mode in modes else 0

    selected_mode = st.selectbox(
        "Select Flight Mode",
        modes,
        index=current_idx
    )

    if st.button(
        "Switch Mode",
        use_container_width=True,
        disabled=not _get_session_var('is_connected', False)
    ):
        st.session_state.ctrl.set_mode(selected_mode)
        st.rerun()

def handle_stop():
    if not _get_session_var('is_connected', False):
        st.warning(MSG_WARN_CONNECTION_REQUIRED)
        return
    st.session_state.movement_mode = 'stopped'
    st.session_state.left_motor = 0
    st.session_state.right_motor = 0
    steering_center()
    direct_motor_command(0, 0)
    st.session_state._steering_reset = True

def direct_motor_command(left_speed, right_speed):
    try:
        st.session_state.ctrl.set_target(left_speed, right_speed)
    except Exception as e:
        st.error(f"Failed to send motor command: {e}")

def steering_center():
    ctrl = st.session_state.ctrl
    ctrl.set_servo(SERVO_LEFT_CHANNEL, STEERING_CENTER_PWM)
    ctrl.set_servo(SERVO_RIGHT_CHANNEL, STEERING_CENTER_PWM)

def render_manual_control():
    st.subheader("🎮 Manual Control")

    _manual_defaults = {
        "manual_dual": 0, "manual_left": 0, "manual_right": 0,
        "_dual_val": 0, "_left_val": 0, "_right_val": 0,
    }
    for _key, _val in _manual_defaults.items():
        if _key not in st.session_state:
            st.session_state[_key] = _val

    # Process STOP request before widgets are instantiated
    if st.session_state.get("_stop_requested", False):
        st.session_state._stop_requested = False
        st.session_state.manual_dual = 0
        st.session_state.manual_left = 0
        st.session_state.manual_right = 0
        st.session_state._dual_val = 0
        st.session_state._left_val = 0
        st.session_state._right_val = 0

    connected = _get_session_var("is_connected", False)
    armed = _get_session_var("is_armed", False)
    disabled = not connected or not armed

    # ---- Dual Propulsion Slider (controls both motors) ----
    dual = st.slider(
        "⚓ Dual Propulsion",
        min_value=-600, max_value=600, step=10,
        key="manual_dual",
        disabled=disabled,
    )
    if dual != st.session_state._dual_val:
        st.session_state._dual_val = dual
        st.session_state._left_val = dual
        st.session_state._right_val = dual
        st.session_state.left_motor = dual
        st.session_state.right_motor = dual
        st.session_state.movement_mode = "straight" if dual != 0 else "stopped"
        st.session_state.manual_left = dual
        st.session_state.manual_right = dual
        if not disabled:
            direct_motor_command(dual, dual)
        st.rerun()

    # ---- Left Motor Slider ----
    left = st.slider(
        "⬅ Left Motor",
        min_value=-600, max_value=600, step=10,
        key="manual_left",
        disabled=disabled,
    )
    if left != st.session_state._left_val:
        st.session_state._left_val = left
        st.session_state.left_motor = left
        if not disabled:
            direct_motor_command(left, st.session_state.right_motor)

    # ---- Right Motor Slider ----
    right = st.slider(
        "➡ Right Motor",
        min_value=-600, max_value=600, step=10,
        key="manual_right",
        disabled=disabled,
    )
    if right != st.session_state._right_val:
        st.session_state._right_val = right
        st.session_state.right_motor = right
        if not disabled:
            direct_motor_command(st.session_state.left_motor, right)

    # ---- STOP Button ----
    if st.button("⛔ STOP", use_container_width=True, type="primary"):
        handle_stop()
        st.session_state._stop_requested = True
        st.rerun()
   

def render_brush_control():
    st.subheader("🧹 Brush Motor Control")

    brush_on = _get_session_var('brush_on', False)
    brush_speed = _get_session_var('brush_speed', 50)

    col1, col2 = st.columns([1, 2])
    with col1:
        new_brush_on = st.toggle(
            "Brush Motor Switch",
            value=brush_on,
            help="Enable/disable brush motors"
        )
    with col2:
        new_brush_speed = st.slider(
            "Speed",
            min_value=0,
            max_value=100,
            value=brush_speed,
            step=1,
            disabled=not new_brush_on,
            help="Brush motor rotation speed (0-100%)"
        )

    if new_brush_on != brush_on or new_brush_speed != brush_speed:
        st.session_state.brush_on = new_brush_on
        st.session_state.brush_speed = new_brush_speed
        st.session_state.ctrl.set_brush(new_brush_on, new_brush_speed)

    status = st.session_state.ctrl.get_status()
    left_pwm = status.get("brush_left_pwm", BRUSH_PWM_CENTER)
    right_pwm = status.get("brush_right_pwm", BRUSH_PWM_CENTER)

    col1, col2, col3 = st.columns(3)
    with col1:
        st.metric(
            f"Left Brush (CH{BRUSH_LEFT_CHANNEL})",
            f"{left_pwm} µs",
            delta="CW" if left_pwm > BRUSH_PWM_CENTER else ("STOP" if left_pwm == BRUSH_PWM_CENTER else "CCW")
        )
    with col2:
        st.metric(
            f"Right Brush (CH{BRUSH_RIGHT_CHANNEL})",
            f"{right_pwm} µs",
            delta="CCW" if right_pwm < BRUSH_PWM_CENTER else ("STOP" if right_pwm == BRUSH_PWM_CENTER else "CW")
        )
    with col3:
        status_text = "🟢 Running" if brush_on else "⚫ Stopped"
        st.metric("Status", status_text)


def render_steering_control():
    st.subheader("🎯 Servo Control (Steering)")

    # Initialize session state defaults for steering
    _steering_defaults = {
        "steering_pwm": STEERING_CENTER_PWM,
        "_steering_val": STEERING_CENTER_PWM,
    }
    for _key, _val in _steering_defaults.items():
        if _key not in st.session_state:
            st.session_state[_key] = _val

    # Process STOP reset before widget instantiation
    if st.session_state.get("_steering_reset", False):
        st.session_state._steering_reset = False
        st.session_state.steering_pwm = STEERING_CENTER_PWM
        st.session_state._steering_val = STEERING_CENTER_PWM

    connected = _get_session_var("is_connected", False)
    disabled = not connected

    steering = st.slider(
        "Steering",
        min_value=500,
        max_value=2500,
        step=5,
        key="steering_pwm",
        disabled=disabled,
        help="500 = Full Left  |  1500 = Center  |  2500 = Full Right",
    )

    if steering != st.session_state._steering_val:
        st.session_state._steering_val = steering

        # Map slider value to left/right servo PWM values
        if steering <= STEERING_CENTER_PWM:
            # Left turn zone (500 → 1500): left servo 1850 → 1500, right stays at 1500
            progress = (STEERING_CENTER_PWM - steering) / 1000.0
            left_pwm = int(STEERING_CENTER_PWM + progress * (STEERING_LEFT_PWM - STEERING_CENTER_PWM))
            right_pwm = STEERING_CENTER_PWM
        else:
            # Right turn zone (1500 → 2500): left stays at 1500, right servo 1500 → 1150
            progress = (steering - STEERING_CENTER_PWM) / 1000.0
            left_pwm = STEERING_CENTER_PWM
            right_pwm = int(STEERING_CENTER_PWM - progress * (STEERING_CENTER_PWM - STEERING_RIGHT_PWM))

        if not disabled:
            ctrl = st.session_state.ctrl
            ctrl.set_servo(SERVO_LEFT_CHANNEL, left_pwm)
            ctrl.set_servo(SERVO_RIGHT_CHANNEL, right_pwm)

        st.rerun()

    # Display current steering position indicator
    if steering == STEERING_CENTER_PWM:
        st.caption("⏺ **Center**")
    elif steering < STEERING_CENTER_PWM:
        pct = int((STEERING_CENTER_PWM - steering) / 1000.0 * 100)
        st.caption(f"⬅ **Left** ({pct}%)")
    else:
        pct = int((steering - STEERING_CENTER_PWM) / 1000.0 * 100)
        st.caption(f"➡ **Right** ({pct}%)")

def render_attitude_control():
    st.subheader("🧭 Attitude Control (GUIDED Mode)")

    if "attitude_angle" not in st.session_state:
        st.session_state.attitude_angle = 0
    if "attitude_thrust_pct" not in st.session_state:
        st.session_state.attitude_thrust_pct = 50

    col1, col2 = st.columns(2)
    with col1:
        angle = st.number_input(
            "Heading Angle (°)",
            min_value=-180,
            max_value=180,
            value=st.session_state.attitude_angle,
            step=1,
            help="0=North, 90=East, ±180=South, -90=West",
        )
        st.session_state.attitude_angle = angle

    with col2:
        thrust_pct = st.slider(
            "Thrust (%)",
            min_value=-100,
            max_value=100,
            value=st.session_state.attitude_thrust_pct,
            step=1,
            help="Positive=Forward, Negative=Backward",
        )
        st.session_state.attitude_thrust_pct = thrust_pct

    direction = ("▲ N" if abs(angle) <= 5 else
                 "▼ S" if abs(abs(angle) - 180) <= 5 else
                 "► E" if abs(angle - 90) <= 5 else
                 "◄ W" if abs(angle + 90) <= 5 else
                 f"{angle}°")
    st.caption(f"Direction: **{direction}**  |  Thrust: **{thrust_pct}%**")

    col1, col2 = st.columns(2)
    with col1:
        if st.button("🚀 Send Attitude", use_container_width=True, type="primary"):
            if _get_session_var("is_connected", False) and _get_session_var("is_armed", False):
                thrust = thrust_pct / 100.0
                st.session_state.ctrl.set_attitude(angle, thrust)
                st.toast(f"Sent: {angle}° @ {thrust:.2f}")
            else:
                st.warning(MSG_WARN_CONNECTION_REQUIRED)

    with col2:
        if st.button("🔄 Reset", use_container_width=True):
            st.session_state.ctrl.reset_attitude()
            st.session_state.attitude_angle = 0
            st.session_state.attitude_thrust_pct = 50
            st.rerun()

def render_sensor_monitor():
    """Main content area — sensor monitoring panel: temperature, humidity, level, motor current."""
    st.subheader("📊 Sensor Monitor")

    # ── Row 1: Environmental data cards ──
    temp = _get_session_var("sensor_temperature")
    hum = _get_session_var("sensor_humidity")
    sht_ok = _get_session_var("sensor_sht3x_ok", False)
    level_val = _get_session_var("sensor_level_value")
    level_unit = _get_session_var("sensor_level_unit", "")
    level_ok = _get_session_var("sensor_level_ok", False)
    bms_conn = _get_session_var("bms_connected", False)
    bms_voltage = _get_session_var("bms_voltage", 0.0)

    col1, col2, col3, col4 = st.columns(4)

    with col1:
        st.metric(
            "🌡 Temperature",
            f"{temp:.1f} °C" if sht_ok and temp is not None else "--",
            delta=None,
        )
    with col2:
        st.metric(
            "💧 Humidity",
            f"{hum:.1f} %" if sht_ok and hum is not None else "--",
            delta=None,
        )
    with col3:
        if level_ok and level_val is not None:
            unit_display = level_unit if level_unit else ""
            st.metric("📏 Level", f"{level_val:.2f} {unit_display}".strip())
        else:
            st.metric("📏 Level", "--")
    with col4:
        st.metric(
            "⚡ System Voltage",
            f"{bms_voltage:.1f} V" if bms_conn else "--",
            delta=None,
        )

    st.divider()

    # ── Row 2: Motor current panel ──
    st.caption("⚡ Motor Current")

    currents = _get_session_var("sensor_currents", {})
    ina_ok = _get_session_var("sensor_ina4230_ok", False)

    # Split channels into left and right sides
    left_channels = [ch for ch in SENSOR_CURRENT_CHANNELS if ch[0] == 0x41]
    right_channels = [ch for ch in SENSOR_CURRENT_CHANNELS if ch[0] != 0x41]

    col_left, col_right = st.columns(2)

    with col_left:
        st.markdown("**🔵 Left Motors**")
        for ch_def in left_channels:
            _render_current_channel(ch_def, currents, ina_ok)

    with col_right:
        st.markdown("**🔴 Right Motors**")
        for ch_def in right_channels:
            _render_current_channel(ch_def, currents, ina_ok)

    # ── Bottom status bar ──
    st.divider()
    status_parts = []
    if sht_ok:
        status_parts.append("🟢 Temp/Hum")
    else:
        status_parts.append("🔴 Temp/Hum")
    if ina_ok:
        status_parts.append("🟢 Current")
    else:
        status_parts.append("🔴 Current")
    if level_ok:
        status_parts.append("🟢 Level")
    else:
        status_parts.append("🔴 Level")

    from datetime import datetime
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    status_parts.append(f"🔄 Updated: {now_str}")

    st.caption("  |  ".join(status_parts))


def _render_current_channel(ch_def, currents: dict, ina_ok: bool):
    """Render a single current channel: progress bar + value + status indicator.

    ch_def: (i2c_addr, channel_number, pwm_label, rated_a, display_name)
    """
    i2c_addr, ch_num, _pwm_label, rated_a, display_name = ch_def
    key = f"0x{i2c_addr:02X}_ch{ch_num}"
    current_val = currents.get(key) if ina_ok else None

    # Determine color and threshold
    if not ina_ok or current_val is None:
        bar_color = "#888888"
        status_icon = "⚫"
        status_text = "Offline"
        display_val = "--"
        progress = 0.0
    else:
        overload_threshold = 0.6 if rated_a >= 5.0 else 0.4
        if current_val > overload_threshold:
            bar_color = "#e74c3c"
            status_icon = "🔴"
            status_text = "Overload"
        elif current_val > overload_threshold * 0.5:
            bar_color = "#f39c12"
            status_icon = "🟠"
            status_text = "Elevated"
        else:
            bar_color = "#2ecc71"
            status_icon = "🟢"
            status_text = "Normal"
        display_val = f"{current_val:.2f} A"
        progress = min(current_val / rated_a, 1.0) if rated_a > 0 else 0.0

    # Single-row layout: label | progress bar | value | status
    cols = st.columns([3, 5, 2, 2])
    with cols[0]:
        st.caption(f"{display_name}")
    with cols[1]:
        bar_html = f"""
        <div style="width:100%; height:14px; background-color:#e0e0e0; border-radius:7px; overflow:hidden;">
            <div style="width:{progress*100:.0f}%; height:100%; background-color:{bar_color}; border-radius:7px; transition: width 0.3s;">
            </div>
        </div>
        """
        st.markdown(bar_html, unsafe_allow_html=True)
    with cols[2]:
        st.caption(display_val)
    with cols[3]:
        st.caption(f"{status_icon} {status_text}")


def render_status(status_placeholder):
    try:
        status = st.session_state.ctrl.get_status()
        with status_placeholder.container():
            st.subheader("📊 Status")
            scol1, scol2, scol3, = st.columns(3)
            scol1.metric("Flight Mode", status.get("flight_mode", "N/A"))
            scol2.metric("Left Motor (CR1)", f"{status['left_motor']:.0f}")
            scol3.metric("Right Motor (CR3)", f"{status['right_motor']:.0f}")

            if st.button("🔄 Refresh Status"):
                sync_controller_status()
                #st.rerun()
    except Exception as e:
        with status_placeholder.container():
            st.error(f"Failed to get status: {e}")

def render_logs(log_placeholder):
    try:
        # Collect logs from all controllers with source tag
        all_logs = []
        all_logs.extend(f"[FC] {entry}" for entry in st.session_state.ctrl.get_logs())
        if "bms" in st.session_state:
            all_logs.extend(f"[BMS] {entry}" for entry in st.session_state.bms.get_logs())
        if "sensor" in st.session_state:
            all_logs.extend(f"[Sensor] {entry}" for entry in st.session_state.sensor.get_logs())

        log_text = "\n".join(reversed(all_logs))
        with log_placeholder.container():
            col1, col2 = st.columns([4, 1])
            with col1:
                st.subheader(f"📋 Log Panel ({len(all_logs)} entries)")
            with col2:
                if st.button("🗑 Clear Logs", use_container_width=True, key="clear_logs"):
                    st.session_state.ctrl.clear_logs()
                    if "bms" in st.session_state:
                        st.session_state.bms.clear_logs()
                    if "sensor" in st.session_state:
                        st.session_state.sensor.clear_logs()
                    st.rerun()
            st.text_area("Logs", value=log_text, height=300, disabled=True)
    except Exception as e:
        with log_placeholder.container():
            st.error(f"Failed to get logs: {e}")
            
def render_divider():
    st.divider()


def render_battery_status():
    st.subheader("🔋 Battery Status")

    bms_connected = _get_session_var("bms_connected", False)
    voltage = _get_session_var("bms_voltage", 0.0)
    soc = _get_session_var("bms_soc", 0.0)
    charging_current = _get_session_var("bms_charging_current", 0.0)
    discharging_current = _get_session_var("bms_discharging_current", 0.0)
    capacity = _get_session_var("bms_capacity", 0.0)

    if bms_connected:
        if soc > 50:
            soc_icon = "🟢"
        elif soc > 20:
            soc_icon = "🟠"
        else:
            soc_icon = "🔴"
        st.markdown(f"**{soc_icon} SOC: {soc:.1f}%**")
    else:
        st.caption("⚫ BMS disconnected")

    col1, col2 = st.columns(2)
    with col1:
        st.metric("Voltage", f"{voltage:.1f} V" if bms_connected else "--")
        st.metric("Charging", f"{charging_current:.1f} A" if bms_connected else "--")
    with col2:
        st.metric("SOC", f"{soc:.1f} %" if bms_connected else "--")
        st.metric("Discharging", f"{discharging_current:.1f} A" if bms_connected else "--")

    st.metric("Capacity", f"{capacity:.1f} Ah" if bms_connected else "--")

    temp_sensors = _get_session_var("bms_temp_sensors", [])
    if bms_connected and temp_sensors:
        st.divider()
        st.caption("[Temperature Sensor details] (Unit: °C / K)")
        lines = []
        for s in temp_sensors:
            idx = s["index"]
            c = s["celsius"]
            k = s["kelvin"]
            sign = "+" if c >= 0 else ""
            lines.append(f"Sensor{idx:02d}: {sign}{c:.1f} °C  ({k:.1f} K)")
        st.code("\n".join(lines), language=None)

def main():
    st.title("🚢 " + PAGE_TITLE)
    init_session_state()
    sync_controller_status()
    sync_bms_status()
    sync_sensor_status()
    render_header()

    with st.sidebar:
        render_system_control()
        st.divider()
        render_mode_selection()
        st.divider()
        render_battery_status()

    status_placeholder = st.empty()
    render_status(status_placeholder)
    render_divider()
    render_sensor_monitor()
    render_divider()
    render_manual_control()
    render_divider()
    render_steering_control()
    render_divider()
    render_brush_control()
    render_divider()
    render_attitude_control()
    render_divider()

    log_placeholder = st.empty()
    render_logs(log_placeholder)
    
if __name__ == "__main__":
    main()