import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import tf2_ros
from px4_msgs.msg import VehicleOdometry


class SlamToVision(Node):
    def __init__(self):
        super().__init__('slam_to_vision')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_link')
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # PX4 uXRCE-DDS topics use BEST_EFFORT
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos)

        self.timer = self.create_timer(0.02, self.on_timer)  # 50 Hz
        self.get_logger().info(
            f'Publishing {self.target_frame}->{self.source_frame} '
            f'to /fmu/in/vehicle_visual_odometry (NED)')

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF not ready: {e}',
                                   throttle_duration_sec=2.0)
            return

        tr = t.transform.translation
        q = t.transform.rotation

        msg = VehicleOdometry()
        now = int(self.get_clock().now().nanoseconds / 1000)  # microseconds
        msg.timestamp = now
        msg.timestamp_sample = now
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED

        # ENU (ROS) -> NED (PX4)
        msg.position = [float(tr.y), float(tr.x), float(-tr.z)]

        # Quaternion ENU->NED. Order in px4_msgs is [w, x, y, z].
        # This swaps x<->y and negates z, matching the position swap.
        msg.q = [float(q.w), float(q.y), float(q.x), float(-q.z)]

        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        msg.velocity = [float('nan')] * 3        # let EKF derive velocity
        msg.angular_velocity = [float('nan')] * 3

        # Tune these to your SLAM accuracy (m^2 and rad^2)
        msg.position_variance = [0.1, 0.1, 0.1]
        msg.orientation_variance = [0.05, 0.05, 0.05]
        msg.velocity_variance = [float('nan')] * 3

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlamToVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
