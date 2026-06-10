#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import sys
import termios
import tty


class RoverOffboardControl(Node):
    def __init__(self):
        super().__init__('rover_offboard_control')
        
        # State variable MUST be initialized first
        self.current_state = State()

        # CRITICAL FIX: Match MAVROS exact QoS settings
        # Publisher uses: RELIABLE + TRANSIENT_LOCAL
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # FIXED: Was VOLATILE
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers - MUST come before waiting for connection
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )

        # Publishers - Using velocity control for rover
        self.velocity_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting...')
        
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set mode service not available, waiting...')

        # Velocity command message
        self.target_velocity = TwistStamped()
        
        # Rover velocity control (m/s)
        self.forward_speed = 0.0
        self.lateral_speed = 0.0
        self.yaw_rate = 0.0
        
        # Speed settings
        self.max_speed = 2.0
        self.speed_increment = 0.2
        self.yaw_rate_increment = 0.3
        
        # Control flags
        self.offboard_enabled = False
        self.armed = False
        self.last_request_time = self.get_clock().now()
        
        # CRITICAL: Wait for Flight Controller connection before starting setpoint stream
        self.get_logger().info('Waiting for Flight Controller connection...')
        
        # Create a rate for checking connection
        check_rate = self.create_rate(5)  # 5 Hz
        timeout_counter = 0
        max_timeout = 50  # 10 seconds at 5 Hz
        
        while rclpy.ok() and not self.current_state.connected:
            rclpy.spin_once(self, timeout_sec=0.1)
            check_rate.sleep()
            timeout_counter += 1
            
            if timeout_counter % 5 == 0:  # Print every 1 second
                self.get_logger().info(
                    f'Waiting for connection... ({timeout_counter // 5}s)'
                )
            
            if timeout_counter >= max_timeout:
                self.get_logger().error('Timeout waiting for FC connection!')
                self.get_logger().error('Please check:')
                self.get_logger().error('  1. MAVROS is running: ros2 node list | grep mavros')
                self.get_logger().error('  2. State topic: ros2 topic echo /mavros/state')
                self.get_logger().error('  3. PX4 SITL is running')
                raise RuntimeError('Flight Controller connection timeout')
        
        self.get_logger().info('✓ Flight Controller connected!')
        
        # CRITICAL: Send initial setpoints BEFORE attempting mode switch
        # PX4 requires setpoint streaming for at least 100ms before accepting OFFBOARD
        self.get_logger().info('Streaming initial setpoints (100 messages at 20Hz)...')
        init_rate = self.create_rate(20)  # 20 Hz
        for i in range(100):
            if not rclpy.ok():
                break
            self.publish_setpoint()
            init_rate.sleep()
            if (i + 1) % 20 == 0:  # Progress update every second
                self.get_logger().info(f'  Sent {i + 1}/100 setpoints...')
        
        self.get_logger().info('✓ Initial setpoint streaming complete!')
        
        # Now start the main timer for continuous publishing (MUST be > 2Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        
        self.get_logger().info('✓ Rover Offboard Control Ready!')
        self.print_instructions()

    def print_instructions(self):
        """Print control instructions"""
        print("\n" + "="*60)
        print("ROVER OFFBOARD VELOCITY CONTROL - KEYBOARD COMMANDS")
        print("="*60)
        print("\nVelocity Control:")
        print("  w : Increase forward speed")
        print("  s : Increase backward speed")
        print("  a : Increase left speed (mecanum only)")
        print("  d : Increase right speed (mecanum only)")
        print("  q : Rotate left (counter-clockwise)")
        print("  e : Rotate right (clockwise)")
        print("  SPACE : STOP all movement")
        print("  x : Reset to zero speed")
        print("\nMode Control:")
        print("  m : Enable OFFBOARD mode")
        print("  n : ARM rover (after OFFBOARD mode)")
        print("  b : DISARM rover")
        print("  l : Switch to MANUAL mode")
        print("\nOther:")
        print("  i : Show current status")
        print("  + : Increase max speed")
        print("  - : Decrease max speed")
        print("  ESC : Exit")
        print("="*60 + "\n")
        print("Ready! Press 'm' to enable OFFBOARD, then 'n' to ARM\n")

    def state_callback(self, msg):
        """Callback for vehicle state updates"""
        prev_mode = self.current_state.mode
        prev_armed = self.current_state.armed
        
        self.current_state = msg
        
        # Log state changes
        if prev_mode != msg.mode:
            self.get_logger().info(f'✓ Mode changed to: {msg.mode}')
            self.offboard_enabled = (msg.mode == "OFFBOARD")
        
        if prev_armed != msg.armed:
            self.armed = msg.armed
            status = "✓ ARMED" if msg.armed else "✗ DISARMED"
            self.get_logger().info(status)

    def publish_setpoint(self):
        """Publish velocity setpoint - separated for reuse"""
        self.target_velocity.header.stamp = self.get_clock().now().to_msg()
        self.target_velocity.header.frame_id = "base_link"
        
        # Set velocity (body frame - relative to rover orientation)
        self.target_velocity.twist.linear.x = self.forward_speed
        self.target_velocity.twist.linear.y = self.lateral_speed
        self.target_velocity.twist.linear.z = 0.0
        
        # Set yaw rate
        self.target_velocity.twist.angular.x = 0.0
        self.target_velocity.twist.angular.y = 0.0
        self.target_velocity.twist.angular.z = self.yaw_rate
        
        self.velocity_pub.publish(self.target_velocity)

    def timer_callback(self):
        """Main control loop - publishes setpoints at 20Hz (MUST be > 2Hz)"""
        # CRITICAL: Continuous setpoint publishing is required to maintain OFFBOARD mode
        # PX4 will exit OFFBOARD if no setpoint received within 500ms
        self.publish_setpoint()

    def handle_keyboard_input(self, key):
        """Handle keyboard input for control"""
        
        if key == 'w':  # Increase forward speed
            self.forward_speed = min(self.forward_speed + self.speed_increment, self.max_speed)
            self.get_logger().info(f'Forward speed: {self.forward_speed:.2f} m/s')
                
        elif key == 's':  # Increase backward speed
            self.forward_speed = max(self.forward_speed - self.speed_increment, -self.max_speed)
            self.get_logger().info(f'Backward speed: {self.forward_speed:.2f} m/s')
                
        elif key == 'a':  # Increase left speed (mecanum)
            self.lateral_speed = min(self.lateral_speed + self.speed_increment, self.max_speed)
            self.get_logger().info(f'Left speed: {self.lateral_speed:.2f} m/s')
                
        elif key == 'd':  # Increase right speed (mecanum)
            self.lateral_speed = max(self.lateral_speed - self.speed_increment, -self.max_speed)
            self.get_logger().info(f'Right speed: {self.lateral_speed:.2f} m/s')
                
        elif key == 'q':  # Rotate left
            self.yaw_rate = min(self.yaw_rate + self.yaw_rate_increment, 1.0)
            self.get_logger().info(f'Yaw rate: {self.yaw_rate:.2f} rad/s (left)')
                
        elif key == 'e':  # Rotate right
            self.yaw_rate = max(self.yaw_rate - self.yaw_rate_increment, -1.0)
            self.get_logger().info(f'Yaw rate: {self.yaw_rate:.2f} rad/s (right)')
                
        elif key == ' ':  # Stop all movement
            self.forward_speed = 0.0
            self.lateral_speed = 0.0
            self.yaw_rate = 0.0
            self.get_logger().info('⏹ STOPPED - All velocities set to zero')
            
        elif key == 'x':  # Reset to zero
            self.forward_speed = 0.0
            self.lateral_speed = 0.0
            self.yaw_rate = 0.0
            self.get_logger().info('↺ Reset - All velocities zeroed')
            
        elif key == '+' or key == '=':  # Increase max speed
            self.max_speed = min(self.max_speed + 0.5, 5.0)
            self.get_logger().info(f'Max speed: {self.max_speed:.2f} m/s')
            
        elif key == '-' or key == '_':  # Decrease max speed
            self.max_speed = max(self.max_speed - 0.5, 0.5)
            self.get_logger().info(f'Max speed: {self.max_speed:.2f} m/s')
            
        elif key == 'm':  # Enable offboard mode
            current_time = self.get_clock().now()
            if self.current_state.mode != "OFFBOARD" and \
               (current_time - self.last_request_time).nanoseconds > 5e9:  # 5 seconds
                self.get_logger().info('Requesting OFFBOARD mode...')
                self.set_offboard_mode()
                self.last_request_time = current_time
            elif self.current_state.mode == "OFFBOARD":
                self.get_logger().info('Already in OFFBOARD mode')
            
        elif key == 'n':  # Arm
            current_time = self.get_clock().now()
            if not self.current_state.armed and \
               (current_time - self.last_request_time).nanoseconds > 5e9:
                if self.current_state.mode == "OFFBOARD":
                    self.get_logger().info('Requesting ARM...')
                    self.arm_vehicle(True)
                    self.last_request_time = current_time
                else:
                    self.get_logger().warn('⚠ Cannot arm: Must be in OFFBOARD mode first (press "m")')
            elif self.current_state.armed:
                self.get_logger().info('Already armed')
            
        elif key == 'b':  # Disarm
            current_time = self.get_clock().now()
            if self.current_state.armed and \
               (current_time - self.last_request_time).nanoseconds > 5e9:
                self.get_logger().info('Requesting DISARM...')
                self.arm_vehicle(False)
                self.last_request_time = current_time
            elif not self.current_state.armed:
                self.get_logger().info('Already disarmed')
            
        elif key == 'l':  # Manual mode
            current_time = self.get_clock().now()
            if (current_time - self.last_request_time).nanoseconds > 5e9:
                self.get_logger().info('Switching to MANUAL mode...')
                self.set_manual_mode()
                self.last_request_time = current_time
            
        elif key == 'i':  # Show status
            self.print_status()
            
        elif key == '\x1b':  # ESC key
            self.get_logger().info('Exit requested')
            return False
            
        return True

    def print_status(self):
        """Print current status"""
        print("\n" + "-"*60)
        print("ROVER STATUS:")
        print(f"  Mode: {self.current_state.mode}")
        print(f"  Armed: {'✓ YES' if self.current_state.armed else '✗ NO'}")
        print(f"  Connected: {'✓ YES' if self.current_state.connected else '✗ NO'}")
        print(f"  Guided: {self.current_state.guided}")
        print(f"  System Status: {self.current_state.system_status}")
        print(f"\nVelocity Control:")
        print(f"  Forward Speed: {self.forward_speed:+.2f} m/s")
        print(f"  Lateral Speed: {self.lateral_speed:+.2f} m/s")
        print(f"  Yaw Rate: {self.yaw_rate:+.2f} rad/s")
        print(f"  Max Speed: {self.max_speed:.2f} m/s")
        print("-"*60 + "\n")

    def set_offboard_mode(self):
        """Switch to OFFBOARD mode"""
        set_mode_req = SetMode.Request()
        set_mode_req.custom_mode = "OFFBOARD"
        
        future = self.set_mode_client.call_async(set_mode_req)
        future.add_done_callback(self.mode_callback)

    def set_manual_mode(self):
        """Switch to MANUAL mode"""
        set_mode_req = SetMode.Request()
        set_mode_req.custom_mode = "MANUAL"
        
        future = self.set_mode_client.call_async(set_mode_req)
        future.add_done_callback(self.mode_callback)

    def mode_callback(self, future):
        """Callback for set mode service"""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('✓ Mode change command accepted')
            else:
                self.get_logger().warn('✗ Mode change command rejected')
        except Exception as e:
            self.get_logger().error(f'✗ Service call failed: {e}')

    def arm_vehicle(self, arm=True):
        """Arm or disarm the vehicle"""
        arm_req = CommandBool.Request()
        arm_req.value = arm
        
        future = self.arming_client.call_async(arm_req)
        future.add_done_callback(self.arm_callback)

    def arm_callback(self, future):
        """Callback for arming service"""
        try:
            response = future.result()
            if response.success:
                action = 'Armed' if response.result else 'Disarmed'
                self.get_logger().info(f'✓ {action} successfully')
            else:
                self.get_logger().warn('✗ Arm/Disarm command rejected')
        except Exception as e:
            self.get_logger().error(f'✗ Service call failed: {e}')


def get_key():
    """Get keyboard input (Unix/Linux)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main(args=None):
    rclpy.init(args=args)
    
    try:
        rover_control = RoverOffboardControl()
    except RuntimeError as e:
        print(f"\n[ERROR] {e}")
        print("\nTroubleshooting steps:")
        print("1. Check MAVROS is running:")
        print("   ros2 node list | grep mavros")
        print("\n2. Check /mavros/state topic:")
        print("   ros2 topic echo /mavros/state")
        print("\n3. Check PX4 SITL is running:")
        print("   ps aux | grep px4")
        rclpy.shutdown()
        return
    
    # Create a separate thread for ROS spinning
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(rover_control,), daemon=True)
    spin_thread.start()
    
    # Main loop for keyboard input
    try:
        while rclpy.ok():
            key = get_key()
            if not rover_control.handle_keyboard_input(key):
                break
    except KeyboardInterrupt:
        pass
    
    # Stop the rover before exiting
    rover_control.forward_speed = 0.0
    rover_control.lateral_speed = 0.0
    rover_control.yaw_rate = 0.0
    rover_control.get_logger().info('⏹ Shutting down - all velocities zeroed')
    
    # Publish final zero velocity
    rover_control.publish_setpoint()
    
    rover_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
