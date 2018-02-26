#!/usr/bin/env python
# controllers: trot_ros, static_walk_ros, freeze, free_gait_impedence_ros
#
# Modes:
# trot_ros: WalkingTrot, Stand
# static_walk: walk, stand

import rospy
from rocoma_msgs.srv import SwitchController as rocomaSwitchController
from anymal_msgs.srv import SwitchController as anymalSwitchController
from joy_manager_msgs.msg import AnyJoy
from quadruped_msgs.srv import *
from free_gait_msgs.srv import *
from free_gait_msgs.msg import ExecuteActionGoal
from geometry_msgs.msg import PoseStamped


def send_action(action):
    rospy.wait_for_service('/free_gait_action_loader/send_action',5)
    try:
        action_srv = rospy.ServiceProxy('/free_gait_action_loader/send_action', SendAction)
        
        sendAction = SendActionRequest()
        sendAction.goal.action_id = action

        resp1 = action_srv(sendAction)
        print "Free Gait Action:", action, resp1.result

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def set_pose(pose):
    print "Setting Base Pose"
    try:
        rospy.wait_for_service('/free_gait_action_loader/move_base/set_pose',5)
    except rospy.ROSException, e:
        print "Wait for Service failed: %s"%e
        return

    try:
        switch_control_srv = rospy.ServiceProxy('/free_gait_action_loader/move_base/set_pose', SetBasePose)
        sbp = SetBasePoseRequest()
        sbp.pose = pose
        resp1 = switch_control_srv(sbp)
        print "Setting Base Pose", resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def switch_controller(controlType):
    print "Switch Controller to", controlType
    rospy.wait_for_service('/anymal_highlevel_controller/switch_controller',5)

    try:
        switch_control_srv = rospy.ServiceProxy('/anymal_highlevel_controller/switch_controller', rocomaSwitchController)
        resp1 = switch_control_srv(controlType)
        print "Switch Controller to", controlType, resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def switch_trot_mode(controlMode):
    rospy.wait_for_service('/trot_ros/go_to_mode',5)
    try:
        switch_mode_srv = rospy.ServiceProxy('/trot_ros/go_to_mode', anymalSwitchController)
        resp1 = switch_mode_srv(controlMode,True)
        print "Switch Mode to Trot:" ,controlMode ,resp1
        return resp1.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def switch_static_walk_mode(controlMode):
    rospy.wait_for_service('/static_walk_ros/go_to_mode',5)
    try:
        switch_mode_srv = rospy.ServiceProxy('/static_walk_ros/go_to_mode', anymalSwitchController)
        resp1 = switch_mode_srv(controlMode,True)
        print "Switch Mode to StaticWalk:" ,controlMode ,resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def send_velocity_message(linearVelocity, angularVelocity):
    velocityPub = rospy.Publisher('/anyjoy/onboard', AnyJoy, queue_size=10)

    msg = AnyJoy()
    t = rospy.get_rostime() #get time as rospy.Time instance
    #print t
    # 0 : lateral in range -1:1
    # 1 : for/bk in range -1:1
    # 2 : unused part of right joiy
    # 3 : turning -1:1
    msg.header.stamp = t
    #msg.joy.axes = [0, m.linear_velocity.x ,0,m.angular_velocity.z]
    msg.joy.axes = [0, linearVelocity, 0, angularVelocity]
    msg.joy.header.stamp = t

    #rospy.loginfo(hello_str)
    velocityPub.publish(msg)