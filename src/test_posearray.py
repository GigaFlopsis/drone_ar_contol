#!/usr/bin/env python
# coding=utf8

import rospy
from geometry_msgs.msg import PoseStamped, Twist, PoseArray, Point, Pose
import copy

from tools import *


pose_msgs = PoseArray()

pose_list = [[0.,0.,1.],
            [1.,-1.,1.5],
            [-3.,1.,0.3]]


if __name__ == '__main__':
    rospy.init_node("pose_pub_node", anonymous=True)
    pose_pub = rospy.Publisher("ar/path", PoseArray, queue_size=1)
    pose_msgs.header.stamp = rospy.Time.now()
    pose_msgs.header.frame_id = "map"
    rospy.Rate(2).sleep()
    for item in pose_list:
        pose = Pose()
        pose.position.x = item[0]
        pose.position.y = item[1]
        pose.position.z = item[2]
        pose.orientation.w = 1
        pose_msgs.poses.append(pose)

    pose_pub.publish(pose_msgs)
    print "print done"



