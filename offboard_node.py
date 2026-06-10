import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg
if __name__ == "__main__":
    rospy.init_node('offboard_test_node', anonymous=True)
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    rospy.loginfo("MAVROS connected!")
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2
    for i in range(100):
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rate.sleep()
    rospy.loginfo("Attempting to set OFFBOARD mode...")
    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"
    if set_mode_client.call(offb_set_mode).mode_sent:
        rospy.loginfo("OFFBOARD mode enabled")
    else:
        rospy.logwarn("Failed to set OFFBOARD mode")
    rospy.loginfo("Attempting to arm...")
    arm_cmd = CommandBool()
    arm_cmd.value = True
    if arming_client.call(arm_cmd).success:
        rospy.loginfo("Vehicle armed")
    else:
        rospy.logerr("Arming failed! This is the problem we need to solve.")
