#!/usr/bin/env python  
import roslib
roslib.load_manifest('nao_basic')
import rospy
from nao_basic.msg import pose
import math
import tf
from tf.transformations import euler_from_quaternion

ROS_TO_MAZE = 168
RAD_TO_DEG = 57.3

if __name__ == '__main__':
    rospy.init_node('tf2pose')
    listener = tf.TransformListener()
    pose_pub = rospy.Publisher('cur_pose', pose)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot1) = listener.lookupTransform('data/multi/patt.letter_a', 'data/multi/patt.letter_b', rospy.Time(0))
            # trans, (dx, dy, dz)
            # rot, (x,y,z,w) should be [theta,0,0,1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        angles = euler_from_quaternion(rot1)
        # roatation round x, y, z axies
        cur_pose = pose()
        cur_pose.x = trans[0]*ROS_TO_MAZE # in cm 
        cur_pose.y = trans[1]*ROS_TO_MAZE # in cm
        cur_pose.theta = angles[2] # in radian
        info = 'X:%s, Y:%s, Theta:%s' %(cur_pose.x, cur_pose.y, cur_pose.theta*RAD_TO_DEG)
        # info = 'trans (in cm): x:%3.5f, y:%3.5f, z:%3.5f \n rot: w: %3.5f, x:%3.5f, y:%3.5f, z:%3.5f' %(trans[0]*ROS_TO_MAZE,
        # trans[1]*ROS_TO_MAZE,trans[2]*ROS_TO_MAZE, rot2[0]*RAD_TO_DEG, rot2[1]*RAD_TO_DEG, rot2[2]*RAD_TO_DEG, rot2[3]*RAD_TO_DEG)
        rospy.loginfo(info)
        pose_pub.publish(cur_pose)
        rate.sleep()
