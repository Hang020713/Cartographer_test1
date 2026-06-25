import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from px4_msgs.msg import RoverThrottleSetpoint, RoverSteeringSetpoint


class CmdVelToRover(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_rover')

        # v_max = the real-world m/s that should map to full throttle (1.0).
        # Measure this on your rover, then set it here.
        self.declare_parameter('max_linear_speed', 0.6)    # m/s -> throttle 1.0
        self.declare_parameter('max_angular_speed', 1.5)   # rad/s -> speed_diff 1.0
        self.declare_parameter('min_throttle', 0.15)       # overcome motor stiction
        self.declare_parameter('cmd_timeout', 0.5)         # stop if no cmd_vel
        self.v_max = self.get_parameter('max_linear_speed').value
        self.w_max = self.get_parameter('max_angular_speed').value
        self.min_throttle = self.get_parameter('min_throttle').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.throttle_pub = self.create_publisher(
            RoverThrottleSetpoint, '/fmu/in/rover_throttle_setpoint', qos)
        self.steering_pub = self.create_publisher(
            RoverSteeringSetpoint, '/fmu/in/rover_steering_setpoint', qos)

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        self.last_v = 0.0
        self.last_w = 0.0
        self.last_stamp = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.on_timer)  # 20 Hz

    def on_cmd(self, msg):
        self.last_v = msg.linear.x
        self.last_w = msg.angular.z
        self.last_stamp = self.get_clock().now()

    def on_timer(self):
        dt = (self.get_clock().now() - self.last_stamp).nanoseconds * 1e-9
        v, w = (0.0, 0.0) if dt > self.cmd_timeout else (self.last_v, self.last_w)

        throttle = max(-1.0, min(1.0, v / self.v_max))
        speed_diff = max(-1.0, min(1.0, w / self.w_max))

        # stiction deadband: don't command a throttle too small to move
        if 1e-3 < abs(throttle) < self.min_throttle:
            throttle = math.copysign(self.min_throttle, throttle)

        t = int(self.get_clock().now().nanoseconds / 1000)

        ts = RoverThrottleSetpoint()
        ts.timestamp = t
        ts.throttle_body_x = float(throttle)
        ts.throttle_body_y = 0.0
        self.throttle_pub.publish(ts)

        ss = RoverSteeringSetpoint()
        ss.timestamp = t
        ss.normalized_speed_diff = float(speed_diff)
        self.steering_pub.publish(ss)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()