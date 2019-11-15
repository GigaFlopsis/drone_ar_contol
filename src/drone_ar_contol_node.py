#!/usr/bin/env python
# coding=utf8
import rospy
from geometry_msgs.msg import PoseStamped, Twist, PoseArray, Point, Pose
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from tools import *
import math
import numpy as np

# + 1. Subscribe on marker avarage and publish to offset rotate
# + 2. subscribe on path from ar glasses
# + 3. send markers of path (markersArray

goal_tolerance = 0.1

init_state = False
index_of_path = int()
index_of_path = -1

poseArray = PoseArray()
local_position = PoseStamped
goal_pose = PoseStamped()
offset_pose = PoseStamped()
offset_marker = Twist()

goal_pose.header.frame_id = "map"
half_pi = math.pi /2
quat_rotate = Quaternion()
pc2_timer = 0.0
pc2_delay = 1.
pc2_old_time = 0.0
def marker_clb(data):
    """
    Get data from found marker topic and sent offset of drone
    :param data:
    :return:
    """
    global offset_marker, quat_rotate

    offset_marker.linear.x = data.pose.position.x
    offset_marker.linear.y = data.pose.position.y
    offset_marker.linear.z = data.pose.position.z

    roll, pitch, yaw = getQuatToEuler(data.pose.orientation.x,
                                      data.pose.orientation.y,
                                      data.pose.orientation.z,
                                      data.pose.orientation.w)

    offset_marker.angular.x = roll - half_pi
    offset_marker.angular.y = pitch
    offset_marker.angular.z = yaw

    # print "offset", math.degrees(offset_marker.angular.x), math.degrees(offset_marker.angular.y), math.degrees(offset_marker.angular.z)

def pose_array_clb(data):
    """
    Get position from hololense
    :param data:
    :return:
    """
    global poseArray, index_of_path, goal_pose
    poseArray = data

    print "get pose", len(data.poses)
    if (len(data.poses) > 0):
        index_of_path = 0

        # Publish fist point
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose = getNormalPose(goal_pose, poseArray.poses[index_of_path])
        goalPub.publish(goal_pose)
    else:
        index_of_path = -1

def nav_pose_clb(data):
    global local_position, init_state, arPosePub, offset_pose, offset_marker

    local_position = data
    offset_pose.header = data.header
    offset_pose.header.stamp = rospy.Time.now()

    rotate_pose = rotate_vect(-offset_marker.angular.z,
                            [data.pose.position.x-offset_marker.linear.x,
                            data.pose.position.y-offset_marker.linear.y,
                            data.pose.position.z-offset_marker.linear.z])

    offset_pose.pose.position.x = rotate_pose[0]
    offset_pose.pose.position.y = rotate_pose[1]
    offset_pose.pose.position.z = rotate_pose[2]

    roll, pitch, yaw = getQuatToEuler(data.pose.orientation.x,
                                      data.pose.orientation.y,
                                      data.pose.orientation.z,
                                      data.pose.orientation.w)

    offset_pose.pose.orientation = getEulerToQuat(roll,pitch,yaw=yaw-offset_marker.angular.z)

    arPosePub.publish(offset_pose)
    init_state = True

def pc2_clb(data):
    """
    :param data:
    :return:
    """

    global pc2_timer, pc2_old_time, pc2_delay, offset_marker, pointsPub
    pc2_timer += rospy.get_time() - old_time
    if pc2_timer < pc2_delay:
        return
    cloud_points = []

    for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
        rotate_pose = rotate_vect(-offset_marker.angular.z,
                                  [p[0] - offset_marker.linear.x,
                                   p[1] - offset_marker.linear.y,
                                   p[2] - offset_marker.linear.z])

        cloud_points.append([rotate_pose[0], rotate_pose[1], rotate_pose[2], 0.0])

    data.data = np.asarray(cloud_points, np.float32).tostring()
    pointsPub.publish(data)

def getNormalPose(local_pose, pose):
    global offset_marker
    # Transform of point relative origin system coords

    rotate_pose = rotate_vect(offset_marker.angular.z,
                              [pose.position.x,
                               pose.position.y,
                               pose.position.z])

    local_pose.pose.position.x = rotate_pose[0] + offset_marker.linear.x
    local_pose.pose.position.y = rotate_pose[1] + offset_marker.linear.y
    local_pose.pose.position.z = rotate_pose[2] + offset_marker.linear.z

    roll, pitch, yaw = getQuatToEuler(pose.orientation.x,
                                      pose.orientation.y,
                                      pose.orientation.z,
                                      pose.orientation.w)

    local_pose.pose.orientation = getEulerToQuat(roll, pitch, yaw=yaw + offset_marker.angular.z)
    return local_pose

if __name__ == '__main__':
    rospy.init_node("drone_ar_control_node", anonymous=True)
    old_time = rospy.get_time()

    pc2_old_time = rospy.get_time()
    # Subscibers
    rospy.Subscriber("/aruco_eye/average_marker", PoseStamped, marker_clb)
    rospy.Subscriber("/ar/path", PoseArray, pose_array_clb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, nav_pose_clb)
    rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, pc2_clb)
    arPosePub = rospy.Publisher("/ar/local_position", PoseStamped, queue_size=10)
    pointsPub = rospy.Publisher("/ar/points", PointCloud2, queue_size=10)

    # Publishers
    goalPub = rospy.Publisher("/goal", PoseStamped, queue_size=10)
    markerArrayPub = rospy.Publisher("/goal/path/markers", MarkerArray, queue_size=10)

    rate = rospy.Rate(5);
    print "init node"
    while rospy.is_shutdown() is False:
        if init_state is False or index_of_path < 0:
            # print "wait of init params"
            continue

        dist_to_goal = getDistPoint(local_position.pose.position, goal_pose.pose.position)
        # print "dist_to_goal",dist_to_goal
        if (dist_to_goal < goal_tolerance and index_of_path < len(poseArray.poses)-1):
            index_of_path += 1
            # Publish next point
            goal_pose.header.stamp = rospy.Time.now()

            goal_pose = getNormalPose(goal_pose, poseArray.poses[index_of_path])

            goalPub.publish(goal_pose)

            print "Publish new pose: ", index_of_path+1, " of ", len(poseArray.poses)

            # print marker array
        goalPub.publish(goal_pose)
        markerArrayPub.publish(getMarkerArray(poseArray))
        rate.sleep()