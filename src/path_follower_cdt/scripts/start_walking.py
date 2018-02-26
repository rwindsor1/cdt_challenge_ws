#!/usr/bin/env python
import sys
import time
import rospy

from high_level_actions import *


rospy.init_node('talker', anonymous=True)

switch_controller("trot_ros")
send_velocity_message(0.0, 0.0)
time.sleep( 5 )

switch_trot_mode("WalkingTrot")
