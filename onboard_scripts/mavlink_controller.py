import math

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
# from config import BRUSH_LEFT_CHANNEL, BRUSH_RIGHT_CHANNEL, SERIAL_PORT, SERIAL_BAUD
import logging
import threading
import time
from collections import deque
from typing import Optional

SERIAL_PORT = "/dev/tty.usbmodem1201"
SERIAL_BAUD = 115200
LOG_MAX_LEN = 100
LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"
SMOOTH_STEP = 20
HEARTBEAT_TIMEOUT = 3.0
RECEIVE_TIMEOUT = 2.0
LOOP_INTERVAL = 0.05
PWM_CENTER = 1500
PWM_MIN = 900
PWM_MAX = 2100
MOTOR_SPEED_MIN = -600
MOTOR_SPEED_MAX = 600


class ListLogHandler(logging.Handler):
    def __init__(self, log_history):
        super().__init__()
        self.log_history = log_history

    def emit(self, record):
        message = self.format(record)
        self.log_history.append(message)


class MavController:
    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD):
        if getattr(self, "initialized", False):
            return

        self.port = port
        self.baud = baud
        self.master: Optional[mavutil.mavlink_connection] = None
        self.log_history = deque(maxlen=LOG_MAX_LEN)
        self.logger = self._setup_logger()
        
        self.flight_mode = "UNKNOWN"
        self.mode_mapping = {}
        self.target_left_motor = 0
        self.target_right_motor = 0
        self.current_left_motor = 0
        self.current_right_motor = 0
        self.direct_angle_deg = 0.0
        self.direct_thrust = 0.0
        self.use_direct_attitude = False
        self.brush_enabled = False
        self.brush_speed_pct = 0
        self.smooth_step = SMOOTH_STEP
        self.last_heartbeat = time.time()
        self.lock = threading.Lock()
        self.running = True
        self._wake_event = threading.Event()
        
        self.is_connected = False
        self.is_armed = False
        self.system_status = "UNKNOWN"

        self._motor_states = {}
        self._motor_lock = threading.Lock()
        self._motor_refresh_thread = None

        self._connect_and_initialize()

        threading.Thread(target=self._send_loop, daemon=True).start()
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()

        self.initialized = True
        print("MavController initialized successfully.")

    def _setup_logger(self) -> logging.Logger:
        logger = logging.getLogger(__name__)
        logger.setLevel(logging.INFO)
        logger.propagate = False

        if not logger.handlers:
            formatter = logging.Formatter(LOG_FORMAT)
            handler = ListLogHandler(self.log_history)
            handler.setFormatter(formatter)
            logger.addHandler(handler)

        return logger

    def _connect_and_initialize(self) -> None:
        try:
            self.logger.info(f"Connecting to flight controller: {self.port} @ {self.baud}")
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
            self._wait_for_heartbeat()
            self.mode_mapping = self.master.mode_mapping() or {}
            self.logger.info(
                f"Flight controller connected (System ID: {self.master.target_system}, Component ID: {self.master.target_component})"
            )
        except Exception as e:
            self.logger.error(f"Failed to connect to flight controller: {e}")
            self.is_connected = False

    def _wait_for_heartbeat(self) -> None:
        self.logger.info("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        self.last_heartbeat = time.time()
    
    def set_mode(self, mode):
        if mode not in self.mode_mapping:
            self.logger.error(f"Unknown flight mode: {mode}")
            return

        mode_id = self.mode_mapping[mode]
        self.logger.info(f"Setting flight mode to: {mode} (ID: {mode_id})")
        try:
            self.master.set_mode(mode_id)
            self.flight_mode = mode
        except Exception as exc:
            self.logger.error(f"Failed to set mode: {exc}")

    def arm(self):
        if not self.master:
            self.logger.error("Cannot ARM: Not connected to the flight controller.")
            return

        self.logger.info("🔐 Sending ARM command...")
        self.master.arducopter_arm()
        time.sleep(1)

    def disarm(self):
        if not self.master:
            self.logger.error("Cannot DISARM: Not connected to the flight controller.")
            return

        self.logger.info("🔓 Sending DISARM command...")
        self.master.arducopter_disarm()
        time.sleep(1)
        self._force_stop()

    def set_target(self, left_motor, right_motor):
        left_motor = max(MOTOR_SPEED_MIN, min(MOTOR_SPEED_MAX, left_motor))
        right_motor = max(MOTOR_SPEED_MIN, min(MOTOR_SPEED_MAX, right_motor))

        with self.lock:
            self.target_left_motor = left_motor
            self.target_right_motor = right_motor
            self.use_direct_attitude = False
        if left_motor != 0 or right_motor != 0:
            self._wake_event.set()
            self.logger.info(f"Setting target: L={left_motor}, R={right_motor}")

    def set_attitude(self, angle_deg: float, thrust: float):
        thrust = max(-1.0, min(1.0, thrust))
        with self.lock:
            self.direct_angle_deg = angle_deg
            self.direct_thrust = thrust
            self.use_direct_attitude = True
            self.logger.info(f"Setting direct attitude: {angle_deg}°, {thrust}")
        self._wake_event.set()

    def reset_attitude(self):
        with self.lock:
            self.current_left_motor = int(self.direct_thrust * 500.0)
            self.current_right_motor = 0
            self.use_direct_attitude = False
            self.direct_angle_deg = 0.0
            self.direct_thrust = 0.0
            self.logger.info("Resetting direct attitude")
        self._wake_event.clear()

    # def set_brush(self, enabled: bool, speed_pct: int):
    #     speed_pct = max(0, min(100, speed_pct))
    #     with self.lock:
    #         self.brush_enabled = enabled
    #         self.brush_speed_pct = speed_pct
    #     left_pwm, right_pwm = self._compute_brush_pwm()
        
    #     self.set_servo(BRUSH_LEFT_CHANNEL, left_pwm)
    #     self.set_servo(BRUSH_RIGHT_CHANNEL, right_pwm)
    #     self.logger.info(f"Brush motors: {'ON' if enabled else 'OFF'}, speed={speed_pct}%, L={left_pwm}us, R={right_pwm}us")

    def _compute_brush_pwm(self):
        if not self.brush_enabled or self.brush_speed_pct == 0:
            return PWM_CENTER, PWM_CENTER
        max_offset = 400  # PWM 1100~1900
        offset = int((self.brush_speed_pct / 100.0) * max_offset)
        left_pwm = PWM_CENTER - offset 
        right_pwm = PWM_CENTER + offset
        return left_pwm, right_pwm

    def stop(self):
        self._force_stop()
        self._wake_event.clear()
        self.logger.info("Stopping...")

    def shutdown(self):
        self.logger.info("Shutting down MavController threads.")
        self.running = False
        self._wake_event.clear()

    def _force_stop(self):
        with self.lock:
            self.target_left_motor = 0
            self.target_right_motor = 0
            self.current_left_motor = 0
            self.current_right_motor = 0
            self.use_direct_attitude = False
            self.brush_enabled = False
            # self.set_servo(BRUSH_LEFT_CHANNEL, PWM_CENTER)
            # self.set_servo(BRUSH_RIGHT_CHANNEL, PWM_CENTER)
            self.rc_channels_override_send(PWM_CENTER, PWM_CENTER)
            self.motor_test_stop()

    def _smooth_update(self, current, target):
        if abs(target - current) <= self.smooth_step:
            return target
        return current + self.smooth_step if target > current else current - self.smooth_step

    def _pwm_from_axis(self, axis_value):
        pwm = int(PWM_CENTER + axis_value)
        return max(PWM_MIN, min(PWM_MAX, pwm))

    def _send_loop(self):
        while self.running:
            if not self.is_connected:
                time.sleep(0.1)
                continue

            with self.lock:
                self.current_left_motor = self._smooth_update(
                    self.current_left_motor, self.target_left_motor
                )
                self.current_right_motor = self._smooth_update(
                    self.current_right_motor, self.target_right_motor
                )

                left_motor = self.current_left_motor
                right_motor = self.current_right_motor
                target_l = self.target_left_motor
                target_r = self.target_right_motor
                use_direct = self.use_direct_attitude
                direct_angle = self.direct_angle_deg
                direct_thrust = self.direct_thrust

            should_stop = (
                (use_direct and direct_thrust == 0.0)
                or (not use_direct and left_motor == 0 and right_motor == 0
                    and target_l == 0 and target_r == 0)
            )

            if should_stop:
                self._wake_event.clear()
                self.logger.info("No active command, entering wait state...")
                self._force_stop()
                self._wake_event.wait()
                continue

            if self.flight_mode == "GUIDED":
                if use_direct:
                    yaw_rad = math.radians(direct_angle)
                    self.send_attitude_target(direct_thrust, yaw_rad)
                else:
                    thrust = left_motor / 500.0
                    yaw_rad = (right_motor / 500.0) * (math.pi / 4)
                    self.send_attitude_target(thrust, yaw_rad)
            else:
                pwm_left = self._pwm_from_axis(left_motor)
                pwm_right = self._pwm_from_axis(right_motor)

                if not self.is_armed:
                    self.stop()
                    continue

                self.rc_channels_override_send(pwm_left, pwm_right)

            time.sleep(LOOP_INTERVAL)

    def _heartbeat_loop(self):
        while self.running:
            try:
                msg = self.master.recv_match(
                    type='HEARTBEAT',
                    blocking=True,
                    timeout=2.0,
                )
            except Exception as exc:
                self.logger.error(f"Heartbeat receive failed: {exc}")
                self.is_connected = False
                time.sleep(1)
                self._force_stop()
                continue

            if not msg:
                if self.is_connected:
                    self.logger.warning("❌ Heartbeat lost")
                self.is_connected = False
                if time.time() - self.last_heartbeat > HEARTBEAT_TIMEOUT:
                    self._force_stop()
                continue

            self.last_heartbeat = time.time()
            self.is_connected = True
            
            if msg.type == mavutil.mavlink.MAV_TYPE_SURFACE_BOAT:
                self.is_armed  = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            
                if self.mode_mapping:
                    for mode, mode_id in self.mode_mapping.items():
                        if getattr(msg, "custom_mode", None) == mode_id:
                            if self.flight_mode != mode:
                                self.flight_mode = mode
                                self.logger.info(f"Flight mode changed to: {mode}")
                            break
    

    def get_status(self) -> dict:
        heartbeat_age = time.time() - self.last_heartbeat
        left_brush, right_brush = self._compute_brush_pwm()
        return {
            "is_connected": self.is_connected,
            "is_armed": self.is_armed,
            "system_status": self.system_status,
            "flight_mode": self.flight_mode,
            "left_motor": self.current_left_motor,
            "right_motor": self.current_right_motor,
            "use_direct_attitude": self.use_direct_attitude,
            "direct_angle_deg": self.direct_angle_deg,
            "heartbeat_age": heartbeat_age,
            "brush_enabled": self.brush_enabled,
            "brush_speed_pct": self.brush_speed_pct,
            "brush_left_pwm": left_brush,
            "brush_right_pwm": right_brush,
        }

    def get_logs(self) -> list:
        return list(self.log_history)

    def clear_logs(self) -> None:
        self.log_history.clear()
        self.logger.info("Log history cleared")

    def set_servo(self, channel, pwm):
        if self.master:
            self.master.set_servo(channel, pwm)

    def set_motor_pwm(self, motor_instance, pwm):
        if not self.master:
            return
        with self._motor_lock:
            self._motor_states[motor_instance] = {'active': True, 'pwm': pwm}
        self._send_motor_command(motor_instance, pwm)
        self._ensure_motor_refresh()

    def _send_motor_command(self, motor_instance, pwm):
        if self.master:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                0,
                motor_instance,
                1,  # throttle_type: 1=PWM
                pwm,
                30,
                0, 0, 0
            )
    
    def _stop_motor_command(self, motor_instance):
        if self.master:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                0,
                motor_instance,
                0,
                0,
                0,
                0, 0, 0
            )

    def _ensure_motor_refresh(self):
        with self._motor_lock:
            if self._motor_refresh_thread and self._motor_refresh_thread.is_alive():
                return
            self._motor_refresh_thread = threading.Thread(
                target=self._motor_refresh_loop, daemon=True
            )
            self._motor_refresh_thread.start()

    def _motor_refresh_loop(self):
        REFRESH_INTERVAL = 25
        while self.running:
            time.sleep(REFRESH_INTERVAL)
            with self._motor_lock:
                motors = [
                    (inst, state['pwm'])
                    for inst, state in self._motor_states.items()
                    if state.get('active')
                ]
            if not motors:
                break
            for inst, pwm in motors:
                self._send_motor_command(inst, pwm)

    def motor_test_stop(self, motor_instance=None):
        if motor_instance is None:
            instances = list(self._motor_states.keys())
        else:
            instances = [motor_instance]
        with self._motor_lock:
            for inst in instances:
                if inst in self._motor_states:
                    self._motor_states[inst]['active'] = False

                if self.master:
                    self._stop_motor_command(inst)
                    time.sleep(0.05)

            for inst, state in self._motor_states.items():
                if state.get('active'):
                    self._send_motor_command(inst, state['pwm'])
    
    def rc_channels_override_send(self, pwm_throttle_left, pwm_throttle_right):
        try:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                pwm_throttle_left,     # chan1
                0,                # chan2
                pwm_throttle_right,     # chan3
                0,                # chan4
                0,                # chan5
                0,                # chan6
                0,                # chan7
                0                 # chan8
            )
        except Exception as exc:
            self.logger.error(f"Failed to send rc override: {exc}")
    
    def get_master(self):
        return self.master

    def send_attitude_target(self, current_thrust, current_yaw_rad):
        q=QuaternionBase([0.0, 0.0, current_yaw_rad])

        self.master.mav.set_attitude_target_send(
            int(time.time() * 1000) & 0xFFFFFFFF,   # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            7,                                      # type_mask = 7 → ignore all body rates, use quaternion
            q,                                      # quaternion [qw, qx, qy, qz]
            0, 0, 0,                                # body rates (ignored)
            current_thrust)                         # thrust = forward speed 
