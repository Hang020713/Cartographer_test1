import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleStatus)


class Offboard(Node):
    def __init__(self):
        super().__init__('offboard')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.offb_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.status_cb, qos)

        self.status = None
        self.counter = 0
        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz
        self.get_logger().info('Offboard node started')

    def status_cb(self, msg):
        self.status = msg

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_heartbeat(self):
        m = OffboardControlMode()
        m.timestamp = self.now_us()
        m.position = False
        m.velocity = True
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.offb_pub.publish(m)

    def publish_velocity(self, vx, vy, yawspeed=0.0):
        sp = TrajectorySetpoint()
        sp.timestamp = self.now_us()
        sp.position = [float('nan')] * 3
        sp.velocity = [float(vx), float(vy), float('nan')]
        sp.acceleration = [float('nan')] * 3
        sp.yaw = float('nan')
        sp.yawspeed = float(yawspeed)
        self.sp_pub.publish(sp)

    def send_command(self, command, p1=0.0, p2=0.0):
        c = VehicleCommand()
        c.timestamp = self.now_us()
        c.command = command
        c.param1 = float(p1)
        c.param2 = float(p2)
        c.target_system = 1
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self.cmd_pub.publish(c)

    def loop(self):
        self.publish_heartbeat()

        if self.counter < 20:
            # warm-up: stream zero velocity before requesting offboard
            self.publish_velocity(0.0, 0.0)
        elif self.counter == 20:
            self.publish_velocity(0.0, 0.0)
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info('Requested OFFBOARD + ARM')
        elif self.counter < 120:
            # ~5 seconds of driving forward at 0.3 m/s
            self.publish_velocity(0.3, 0.0)
        else:
            # stop
            self.publish_velocity(0.0, 0.0)

        if self.status is not None and self.counter % 20 == 0:
            self.get_logger().info(
                f'nav_state={self.status.nav_state}  '
                f'arming_state={self.status.arming_state}')

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = Offboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()