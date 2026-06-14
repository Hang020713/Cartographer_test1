import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import SensorCombined, VehicleAttitude
from sensor_msgs.msg import Imu


class Px4ImuBridge(Node):
    def __init__(self):
        super().__init__('px4_imu_bridge')
        print("Start px4_imu_bridge node")

        # QoS that matches the PX4 uXRCE-DDS publisher
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.latest_q = None  # store latest attitude

        self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined',
            self.sensor_cb, px4_qos)

        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.attitude_cb, px4_qos)

        self.pub = self.create_publisher(Imu, '/imu/data', 10)

    def attitude_cb(self, msg):
        # PX4 quaternion is [w, x, y, z] in NED
        self.latest_q = msg.q

    def sensor_cb(self, msg):
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'   # match your URDF / Cartographer config

        # FRD -> FLU : x stays, y and z are negated
        imu.angular_velocity.x =  float(msg.gyro_rad[0])
        imu.angular_velocity.y = -float(msg.gyro_rad[1])
        imu.angular_velocity.z = -float(msg.gyro_rad[2])

        imu.linear_acceleration.x =  float(msg.accelerometer_m_s2[0])
        imu.linear_acceleration.y = -float(msg.accelerometer_m_s2[1])
        imu.linear_acceleration.z = -float(msg.accelerometer_m_s2[2])

        if self.latest_q is not None:
            # NED [w,x,y,z] -> ENU FLU orientation
            w, x, y, z = self.latest_q
            imu.orientation.w = float(w)
            imu.orientation.x = float(x)
            imu.orientation.y = -float(y)
            imu.orientation.z = -float(z)
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
