import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros


class SlamToVision(Node):
    def __init__(self):
        super().__init__('slam_to_vision')

        # Make frames configurable so you don't have to rebuild to change them
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_link')
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.timer = self.create_timer(0.02, self.on_timer)   # 50 Hz

        self.get_logger().info(
            f'Publishing {self.target_frame} -> {self.source_frame} to /mavros/vision_pose/pose')

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF not ready: {e}', throttle_duration_sec=2.0)
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = t.transform.translation.z
        msg.pose.orientation = t.transform.rotation
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
