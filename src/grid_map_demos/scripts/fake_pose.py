#!/usr/bin/env python  
import roslib
import rospy
import tf
import time


def send_tf_pose():
    print "run"
    # x,y,z
    # r,p,y
    br.sendTransform((-0.25, 1.25, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "/base",
                     "/odom")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    br = tf.TransformBroadcaster()

    while (1==1):
        print "nerf"
        send_tf_pose()
        time.sleep(0.1)