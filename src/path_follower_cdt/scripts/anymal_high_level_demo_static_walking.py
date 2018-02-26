#!/usr/bin/env python
import sys
import time
import rospy

from high_level_actions import *


rospy.init_node('talker', anonymous=True)

#####################################
switch_controller("static_walk_ros")
send_velocity_message(0.0, 0.0)
time.sleep( 10 )

print "static walking"
switch_static_walk_mode("walk")
#send_velocity_message(-0.3, 0.0)
#time.sleep( 15 )