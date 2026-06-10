#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Header


class TeleopToMavros(Node):
    def __init__(self):
        super().__init__('teleop_to_mavros')
        
        # Subscribe to cmd_vel from teleop_twist_keyboard
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish RC override commands to MAVROS
        self.rc_override_pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )
        
        # Parameters for scaling
        self.declare_parameter('max_linear_speed', 1.0)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        
        # RC channel values (1000-2000, 1500 is neutral)
        self.rc_min = 1000
        self.rc_max = 2000
        self.rc_neutral = 1000
        
        self.get_logger().info('Teleop to MAVROS bridge started')
        self.get_logger().info('Listening to /cmd_vel and publishing to /mavros/rc/override')

    def cmd_vel_callback(self, msg):
        # Get parameters
        max_linear = self.get_parameter('max_linear_speed').value
        max_angular = self.get_parameter('max_angular_speed').value
        
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Normalize velocities
        linear_norm = max(min(linear_x / max_linear, 1.0), -1.0) if max_linear > 0 else 0.0
        angular_norm = max(min(angular_z / max_angular, 1.0), -1.0) if max_angular > 0 else 0.0
        
        # For a differential drive rover:
        # Channel 1 (Roll): Steering
        # Channel 3 (Throttle): Forward/Backward
        
        # Calculate throttle (channel 3) - forward/backward
        throttle = self.rc_neutral + int(linear_norm * 500)
        
        # Calculate steering (channel 1) - left/right
        steering = self.rc_neutral + int(angular_norm * 500)
        
        # Create RC override message
        rc_msg = OverrideRCIn()
        rc_msg.channels = [1500] * 18  # Initialize all channels to 0 (no override)
        
        # Set the channels (index 0 = channel 1, index 2 = channel 3)
        rc_msg.channels[0] = steering   # Channel 1: Steering
        rc_msg.channels[2] = throttle   # Channel 3: Throttle
        
        # Publish
        self.rc_override_pub.publish(rc_msg)
        
        self.get_logger().info(
            f'Linear: {linear_x:.2f} m/s, Angular: {angular_z:.2f} rad/s | '
            f'Throttle: {throttle}, Steering: {steering}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TeleopToMavros()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Release RC override on shutdown
        rc_msg = OverrideRCIn()
        rc_msg.channels = [0] * 18
        node.rc_override_pub.publish(rc_msg)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
