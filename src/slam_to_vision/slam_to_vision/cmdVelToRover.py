#!/usr/bin/env python3
"""
PX4 rover offboard via RoverThrottleSetpoint + RoverSteeringSetpoint.

CONTROL_MODE:
  "test"   -> ignore everything, drive a fixed pattern (motor path test)
  "vision" -> sphere-tracking control (needs camera + cv_bridge + cv2)
  "nav2"   -> follow /cmd_vel (geometry_msgs/Twist) from Nav2

Verify FIRST that rover_throttle_setpoint and rover_steering_setpoint are
listed under `subscriptions:` in dds_topics.yaml, or the /fmu/in messages
go nowhere.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)

from px4_msgs.msg import (
    OffboardControlMode,
    RoverThrottleSetpoint,
    RoverSteeringSetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)

from geometry_msgs.msg import Twist


CONTROL_MODE = "nav2"      # "test" | "vision" | "nav2"
IMG_WIDTH = 640

# ---- Nav2 cmd_vel -> rover scaling (tune these) ----
MAX_LINEAR_SPEED = 0.6     # m/s that maps to throttle = 1.0
MAX_ANGULAR_SPEED = 1.5    # rad/s that maps to steer = 1.0
MIN_THROTTLE = 0.15        # overcome motor stiction
CMD_TIMEOUT = 0.5          # stop if no cmd_vel for this long (s)


def px4_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class RoverThrottleControl(Node):
    def __init__(self):
        super().__init__('rover_throttle_control')

        qos = px4_qos()

        # ---- Publishers (PX4 inputs) ----
        self.ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.throttle_pub = self.create_publisher(
            RoverThrottleSetpoint, '/fmu/in/rover_throttle_setpoint', qos)
        self.steer_pub = self.create_publisher(
            RoverSteeringSetpoint, '/fmu/in/rover_steering_setpoint', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ---- Subscribers (PX4 outputs) for feedback ----
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self.status_cb, qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_pos_cb, qos)

        # ---- State ----
        self.nav_state = None
        self.arming_state = None
        self.armed = False
        self.in_offboard = False
        self.status_received = False

        self.counter = 0
        self.elapsed = 0.0
        self.dt = 0.05  # 20 Hz

        # ---- Vision state ----
        self.cx = None
        self.depth = None
        self.prev_error = 0.0

        # ---- Nav2 state ----
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.cmd_stamp = self.get_clock().now()

        if CONTROL_MODE == "vision":
            # Camera deps only imported when actually needed
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge
            import cv2
            import numpy as np
            self._cv2 = cv2
            self._np = np
            self.bridge = CvBridge()
            self.depth_image = None
            self.create_subscription(
                Image, '/rover/rgb_camera', self.rgb_callback, 10)
            self.create_subscription(
                Image, '/rover/depth_camera', self.depth_callback, 10)

        elif CONTROL_MODE == "nav2":
            # Nav2 publishes /cmd_vel with default (reliable) QoS
            self.create_subscription(
                Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f"Rover throttle control running — CONTROL_MODE={CONTROL_MODE}")

    # ---------------- Feedback callbacks ----------------
    def status_cb(self, msg: VehicleStatus):
        self.status_received = True
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.in_offboard = (
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def local_pos_cb(self, msg: VehicleLocalPosition):
        pass  # available if you want odometry feedback

    # ---------------- Nav2 ----------------
    def cmd_vel_cb(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.cmd_stamp = self.get_clock().now()

    # ---------------- Vision ----------------
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    def detect_sphere(self, frame):
        cv2, np = self._cv2, self._np
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (11, 11), 2)
        circles = cv2.HoughCircles(
            blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=80,
            param1=50, param2=25, minRadius=20, maxRadius=300)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            return max(circles[0], key=lambda c: c[2])
        return None

    def rgb_callback(self, msg):
        np = self._np
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = frame.shape
        result = self.detect_sphere(frame)
        if result is not None and self.depth_image is not None:
            cx, cy, r = result
            dh, dw = self.depth_image.shape
            cx_d = int(np.clip(cx * (dw - 1) / w, 0, dw - 1))
            cy_d = int(np.clip(cy * (dh - 1) / h, 0, dh - 1))
            depth = self.depth_image[cy_d, cx_d]
            if not np.isnan(depth) and depth > 0:
                self.cx = cx
                self.depth = float(depth)

    # ---------------- Command helpers ----------------
    def now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def publish_ocm(self):
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()
        # Rover throttle = thrust along body-x, steering = torque about body-z
        msg.thrust_and_torque = True
        self.ocm_pub.publish(msg)

    def publish_setpoint(self, throttle: float, steer: float):
        thr = RoverThrottleSetpoint()
        thr.timestamp = self.now_us()
        thr.throttle_body_x = float(max(-1.0, min(1.0, throttle)))
        # body_y is mecanum-only; NaN if not mecanum
        thr.throttle_body_y = float('nan')
        self.throttle_pub.publish(thr)

        st = RoverSteeringSetpoint()
        st.timestamp = self.now_us()
        st.normalized_steering_setpoint = float(max(-1.0, min(1.0, steer)))
        self.steer_pub.publish(st)

    def arm(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def set_offboard(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def send_cmd(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    # ---------------- Control logic ----------------
    def compute_test_drive(self):
        """Fixed pattern: 0-3 s forward, 3-6 s forward + right turn, then stop."""
        t = self.elapsed
        if t < 3.0:
            return 0.5, 0.0
        elif t < 6.0:
            return 0.5, 0.4
        else:
            return 0.0, 0.0

    def compute_tracking(self):
        np = self._np
        if self.cx is None:
            return 0.0, 0.0
        error = (self.cx - IMG_WIDTH / 2) / (IMG_WIDTH / 2)
        error = 0.6 * self.prev_error + 0.4 * error
        self.prev_error = error
        steer = np.clip(0.5 * error, -1.0, 1.0)
        if abs(error) < 0.05:
            steer = 0.0
        if self.depth is None:
            return 0.0, 0.0
        if self.depth <= 1.0:
            return 0.0, 0.0
        elif self.depth > 2.0:
            base = 0.6
        elif self.depth > 1.2:
            base = 0.4
        else:
            base = 0.0
        throttle = base * (1 - 1.5 * abs(error))
        throttle = max(0.3, throttle)  # deadzone, only once moving is intended
        return throttle, steer

    def compute_nav2(self):
        """Convert latest /cmd_vel into normalized throttle + steering."""
        dt = (self.get_clock().now() - self.cmd_stamp).nanoseconds * 1e-9
        if dt > CMD_TIMEOUT:
            return 0.0, 0.0  # stale command -> stop (safety)

        v, w = self.cmd_v, self.cmd_w

        throttle = max(-1.0, min(1.0, v / MAX_LINEAR_SPEED))
        steer = max(-1.0, min(1.0, w / MAX_ANGULAR_SPEED))

        # stiction deadband: don't command a throttle too small to move
        if 1e-3 < abs(throttle) < MIN_THROTTLE:
            throttle = math.copysign(MIN_THROTTLE, throttle)

        return throttle, steer

    def control_loop(self):
        # 1) Always stream the heartbeat.
        self.publish_ocm()

        # 2) Preflight: stream zero setpoints for ~1 s before arming.
        if self.counter < 20:
            self.publish_setpoint(0.0, 0.0)
            self.counter += 1
            return

        # 3) Arm + offboard once, then retry if not confirmed.
        if self.counter == 20:
            self.arm()
            self.set_offboard()
        if self.counter % 20 == 0 and not (self.armed and self.in_offboard):
            self.arm()
            self.set_offboard()

        self.counter += 1
        self.elapsed += self.dt

        # 4) Don't drive until offboard is actually confirmed.
        if not (self.armed and self.in_offboard):
            self.publish_setpoint(0.0, 0.0)
            if self.counter % 20 == 0:
                self.get_logger().warn(
                    f"Waiting for offboard: nav_state={self.nav_state} "
                    f"armed={self.armed} (need nav_state=14, armed=True)")
            return

        # 5) Compute and publish command.
        if CONTROL_MODE == "test":
            throttle, steer = self.compute_test_drive()
        elif CONTROL_MODE == "vision":
            throttle, steer = self.compute_tracking()
        else:  # "nav2"
            throttle, steer = self.compute_nav2()
        self.publish_setpoint(throttle, steer)

        if self.counter % 10 == 0:
            self.get_logger().info(
                f"nav={self.nav_state} armed={self.armed} "
                f"throttle={throttle:.2f} steer={steer:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = RoverThrottleControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
