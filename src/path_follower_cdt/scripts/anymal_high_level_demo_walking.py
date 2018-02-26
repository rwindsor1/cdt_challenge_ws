#!/usr/bin/env python
import sys
import time
import rospy

from high_level_actions import *


rospy.init_node('talker', anonymous=True)

switch_controller("trot_ros")
send_velocity_message(0.0, 0.0)
time.sleep( 10 )

switch_trot_mode("WalkingTrot")
send_velocity_message(0.3, 0.0)
time.sleep( 5 )

send_velocity_message(0.0, -0.2)
time.sleep( 5 )

send_velocity_message(0.0, 0.0)
switch_trot_mode("Stand")
time.sleep( 10 )

#####################################
switch_controller("static_walk_ros")
send_velocity_message(0.0, 0.0)
time.sleep( 10 )

switch_static_walk_mode("walk")
send_velocity_message(-0.3, 0.0)
time.sleep( 15 )
send_velocity_message(0.0, 0.2)
time.sleep( 5 )

send_velocity_message(0., 0.)
switch_static_walk_mode("stand")
send_velocity_message(0., 0.)