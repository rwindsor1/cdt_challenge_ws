#!/usr/bin/env python
import sys
import time
import rospy
from high_level_actions import *
from geometry_msgs.msg import PoseStamped

rospy.init_node('talker', anonymous=True)

switch_controller("free_gait_impedance_ros")
time.sleep( 5 )

