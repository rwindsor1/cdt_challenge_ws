#! /usr/bin/env python
import time
import rospy

# ROS messages
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from joy_manager_msgs.msg import AnyJoy
from anymal_msgs.srv import SwitchController as anymalSwitchController

def on_stop_walking(data):
    print "Received Stop Walking message"
    switch_trot_mode("stand")

def on_base_velocity_command(data):
    t = rospy.get_rostime()
    msg = TwistStamped()
    msg.header.stamp = t
    msg.header.frame_id = 'base'
    msg.twist = data

    velocity_pub.publish(msg)

def switch_trot_mode(controlMode):
    print "Switch Trot mode to", controlMode
    # attempt to catch this exception:
    try:
        rospy.wait_for_service('/trot_ros/go_to_mode',5)
    except rospy.ServiceException, e:
        print "Wait for service failed: %s"%e
        return

    try:
        switch_mode_srv = rospy.ServiceProxy('/trot_ros/go_to_mode', anymalSwitchController)
        resp1 = switch_mode_srv(controlMode,True)
        print "Switch Mode to Trot:" ,controlMode ,resp1
        return resp1.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


####################################################################
rospy.init_node('posture_interface', anonymous=True)
velocity_pub = rospy.Publisher('/joy_manager/twist', TwistStamped, queue_size=10)
print "posture_interface started"

#switch_trot_mode("walk")
rospy.Subscriber("/stop_walking_cmd", Int16, on_stop_walking)
rospy.Subscriber("/position_controller/position_controller_cmd", Twist, on_base_velocity_command)

rospy.spin()
