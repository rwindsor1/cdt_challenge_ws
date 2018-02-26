#!/usr/bin/env python
import sys
import time
import rospy

from high_level_actions import *


rospy.init_node('talker', anonymous=True)

send_velocity_message(0.0, 0.0)
switch_trot_mode("Stand")
