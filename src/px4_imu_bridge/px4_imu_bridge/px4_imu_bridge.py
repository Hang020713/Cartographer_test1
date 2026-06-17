import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import SensorCombined, VehicleAttitude
from sensor_msgs.msg import Imu # ROS, https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html


# PX4 is using FRD, and ROS is using FLU
# FRD: Front-Right-Down
# FLU: Front-Left-Up

def px4_quat_to_ros(q_px4):
    """Convert PX4 attitude quaternion (NED-FRD, [w,x,y,z])
    to ROS orientation (ENU-FLU, [w,x,y,z])."""
    w, x, y, z = q_px4
    a = math.sqrt(0.5)
    rw = -a * (w + z)
    rx = -a * (x + y)
    ry =  a * (y - x)
    rz =  a * (z - w)
    return rw, rx, ry, rz

class Px4ImuBridge(Node):
    def __init__(self):
        super().__init__('px4_imu_bridge')
        self.get_logger().info("Start px4_imu_bridge node")

        # QoS that matches the PX4 uXRCE-DDS publisher
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # UDP, ROS2 default is reliable, so need to be BEST_EFFORT
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.latest_q = None          # store latest attitude
        self.clock_offset_ns = None   # companion_clock - fc_clock, in ns

        # Subscribe /fmu/out/sensor_combined & /fmu/out/vehicle_attitude
        self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined',
            self.sensor_cb, px4_qos)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.attitude_cb, px4_qos)

        # Create publisher /imu/data
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

    # Store the latest quaternion
    def attitude_cb(self, msg):
        # PX4 quaternion is [w, x, y, z] in NED-FRD
        self.latest_q = msg.q

    def fc_stamp_to_ros(self, fc_timestamp_us):
        """Map an FC timestamp (microseconds since FC boot) into the
        companion ROS clock, using a one-time locked offset."""
        fc_time_ns = int(fc_timestamp_us) * 1000    # Convert to ns
        now_ns = self.get_clock().now().nanoseconds

        if self.clock_offset_ns is None:
            # Lock the offset on the first sample.
            self.clock_offset_ns = now_ns - fc_time_ns
            self.get_logger().info(
                f"Locked FC->companion clock offset: "
                f"{self.clock_offset_ns * 1e-9:.6f} s")

        corrected_ns = fc_time_ns + self.clock_offset_ns
        return Time(nanoseconds=corrected_ns).to_msg()

    def sensor_cb(self, msg):
        imu = Imu()

        # Use the FC measurement time mapped into companion clock.
        if msg.timestamp != 0:
            imu.header.stamp = self.fc_stamp_to_ros(msg.timestamp)
        else:
            # Fallback if FC timestamp is missing for some reason.
            imu.header.stamp = self.get_clock().now().to_msg()
    
        imu.header.frame_id = 'imu_link'   # must exist in your TF tree

        # FRD -> FLU : x stays, y and z are negated
        imu.angular_velocity.x =  float(msg.gyro_rad[0])
        imu.angular_velocity.y = -float(msg.gyro_rad[1])    # Opposite
        imu.angular_velocity.z = -float(msg.gyro_rad[2])    # Opposite

        imu.linear_acceleration.x =  float(msg.accelerometer_m_s2[0])
        imu.linear_acceleration.y = -float(msg.accelerometer_m_s2[1])    # Opposite
        imu.linear_acceleration.z = -float(msg.accelerometer_m_s2[2])    # Opposite

        # If received IMU data then process
        if self.latest_q is not None:
            w, x, y, z = px4_quat_to_ros(self.latest_q)
            imu.orientation.w = w
            imu.orientation.x = x
            imu.orientation.y = y
            imu.orientation.z = z
        else:
            # tell consumers orientation is invalid
            imu.orientation_covariance[0] = -1.0

        self.pub.publish(imu)


def main():
    rclpy.init()
    node = Px4ImuBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()