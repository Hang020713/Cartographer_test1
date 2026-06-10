#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

class OdometryBridge : public rclcpp::Node
{
public:
    OdometryBridge() : Node("odometry_bridge")
    {
        // Subscribe to ROS2 odometry from Cartographer
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&OdometryBridge::odom_callback, this, std::placeholders::_1));

        // Publish to PX4
        visual_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);

        RCLCPP_INFO(this->get_logger(), "Odometry Bridge Node Started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /odom");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /fmu/in/vehicle_visual_odometry");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Convert ROS2 Odometry to PX4 VehicleOdometry
        px4_msgs::msg::VehicleOdometry px4_odom;

        // Timestamp (microseconds)
        px4_odom.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        px4_odom.timestamp_sample = px4_odom.timestamp;

        // Position (NED frame for PX4)
        // ROS2 uses ENU (East-North-Up), PX4 uses NED (North-East-Down)
        px4_odom.position[0] = msg->pose.pose.position.y;   // North = ROS Y
        px4_odom.position[1] = msg->pose.pose.position.x;   // East = ROS X
        px4_odom.position[2] = -msg->pose.pose.position.z;  // Down = -ROS Z

        // Velocity (NED frame)
        px4_odom.velocity[0] = msg->twist.twist.linear.y;   // North velocity
        px4_odom.velocity[1] = msg->twist.twist.linear.x;   // East velocity
        px4_odom.velocity[2] = -msg->twist.twist.linear.z;  // Down velocity

        // Quaternion (convert ENU to NED)
        // ROS quaternion is in ENU frame, need to convert to NED
        tf2::Quaternion q_enu(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        // Rotation from ENU to NED: 90° around Z, then 180° around X
        tf2::Quaternion q_enu_to_ned;
        q_enu_to_ned.setRPY(M_PI, 0, M_PI_2);
        tf2::Quaternion q_ned = q_enu_to_ned * q_enu;

        px4_odom.q[0] = q_ned.w();
        px4_odom.q[1] = q_ned.x();
        px4_odom.q[2] = q_ned.y();
        px4_odom.q[3] = q_ned.z();

        // Angular velocity (might need frame conversion too)
        px4_odom.angular_velocity[0] = msg->twist.twist.angular.y;
        px4_odom.angular_velocity[1] = msg->twist.twist.angular.x;
        px4_odom.angular_velocity[2] = -msg->twist.twist.angular.z;

        // Covariances (if available, otherwise set to default values)
        if (msg->pose.covariance[0] > 0) {
            px4_odom.position_variance[0] = msg->pose.covariance[0];  // x variance
            px4_odom.position_variance[1] = msg->pose.covariance[7];  // y variance
            px4_odom.position_variance[2] = msg->pose.covariance[14]; // z variance
        } else {
            // Default variance if not provided
            px4_odom.position_variance[0] = 0.1f;
            px4_odom.position_variance[1] = 0.1f;
            px4_odom.position_variance[2] = 0.1f;
        }

        if (msg->twist.covariance[0] > 0) {
            px4_odom.velocity_variance[0] = msg->twist.covariance[0];
            px4_odom.velocity_variance[1] = msg->twist.covariance[7];
            px4_odom.velocity_variance[2] = msg->twist.covariance[14];
        } else {
            px4_odom.velocity_variance[0] = 0.1f;
            px4_odom.velocity_variance[1] = 0.1f;
            px4_odom.velocity_variance[2] = 0.1f;
        }

        // Set frame and quality
        px4_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        px4_odom.quality = 100; // Good quality

        // Publish
        visual_odom_pub_->publish(px4_odom);

        // Debug output (every 50 messages to avoid spam)
        static int count = 0;
        if (++count % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "Published odometry - Pos: [%.2f, %.2f, %.2f], Vel: [%.2f, %.2f, %.2f]",
                px4_odom.position[0], px4_odom.position[1], px4_odom.position[2],
                px4_odom.velocity[0], px4_odom.velocity[1], px4_odom.velocity[2]);
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odom_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryBridge>());
    rclcpp::shutdown();
    return 0;
}
