#!/usr/bin/env python
# Note: this demos can only be done when the robot is at the origin
# as the pose must be reachable from the standing configuration
# otherwise the robot will fall over
import sys
import time
import rospy
from high_level_actions import *
from geometry_msgs.msg import PoseStamped

rospy.init_node('talker', anonymous=True)

switch_controller("free_gait_impedance_ros")
time.sleep( 10 )

send_action("square_up")
time.sleep( 10 )

send_action("move_base")
print "Move Base"
time.sleep( 2 )

pose = PoseStamped()
t = rospy.get_rostime()
pose.header.stamp = t
pose.header.frame_id = 'map'
pose.pose.position.x = 0
pose.pose.position.y = 0        
pose.pose.position.z = 0.35
pose.pose.orientation.w = 1
set_pose(pose)