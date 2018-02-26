#! /usr/bin/env python
import time
import rospy
from high_level_actions import *

# ROS messages
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from joy_manager_msgs.msg import AnyJoy

global base_velocity_control_enabled
base_velocity_control_enabled = False

def on_body_posture_goal(data):
    print "Received Body Plan message"
    switch_controller("free_gait_impedance_ros")
    time.sleep( 2 )
    send_action("move_base")
    time.sleep( 2 )
    set_pose(data)

def on_stop_walking(data):
    print "Received Stop Walking message"
    # stand static_walk | Stand trot_ros
    switch_trot_mode("Stand")

def on_committed_walking_plan(data):
    global base_velocity_control_enabled

    print "Received Walking Plan message"
    switch_controller("trot_ros")
    switch_trot_mode("WalkingTrot")
    print "Enabling velocity commands"
    base_velocity_control_enabled = True


def on_base_velocity_command(data):
    global base_velocity_control_enabled
    if base_velocity_control_enabled is False:
        print "Ignoring velocity command - not in walking trot mode"
        return
   
    msg = AnyJoy()
    t = rospy.get_rostime() #get time as rospy.Time instance
    #print t

    # 0 : lateral in range -1:1
    # 1 : for/bk in range -1:1
    # 2 : unused part of right joiy
    # 3 : turning -1:1
    msg.header.stamp = t
    msg.joy.axes = [0, data.linear.x ,0,data.angular.z]
    msg.joy.header.stamp = t

    velocity_pub.publish(msg)


####################################################################
rospy.init_node('posture_interface', anonymous=True)
velocity_pub = rospy.Publisher('/anyjoy/onboard', AnyJoy, queue_size=10)
print "posture_interface started"

rospy.Subscriber("/body_posture_goal", PoseStamped, on_body_posture_goal) # works and tested
rospy.Subscriber("/stop_walking_cmd", Int16, on_stop_walking)
# temporary trigger to start walking
rospy.Subscriber("/footstep_plan_request", PoseStamped, on_committed_walking_plan)
rospy.Subscriber("/goal", PoseStamped, on_committed_walking_plan)
rospy.Subscriber("/path_follower/path_follower_cmd", Twist, on_base_velocity_command)

rospy.spin()
