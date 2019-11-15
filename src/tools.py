#!/usr/bin/env python
# coding=utf8
import tf
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import rospy
import numpy as np

def getQuatToEuler(x, y, z, w):
    """
    Transform quaternion to euler angels
    :param x:
    :param y:
    :param z:
    :param w:
    :return: euler angels
    """
    # type(pose) = geometry_msgs.msg.Pose
    euler = tf.transformations.euler_from_quaternion((x,y,z,w))
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return roll,pitch,yaw

def getEulerToQuat(roll=0., pitch=0., yaw = 0.):
    """
    Transform euler angels to quaternion
    :param roll:
    :param pitch:
    :param yaw:
    :return: quaternion
    """
    # type(pose) = geometry_msgs.msg.Pose
    q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    quat = Quaternion()
    quat.x = q[0]
    quat.y = q[1]
    quat.z = q[2]
    quat.w = q[3]
    return quat

def getDistPoint(pose_1, pose_2, use2d=False, getHeight = False):
    """
    Get dist between 2 Point
    :param pose_1: Vector 1 of Point()
    :type pose_1: Point
    :param pose_2: Vector 2 of Point()
    :type pose_2: Point
    :return: distance
    """
    if getHeight:
        return np.abs(pose_2.z-pose_1.z)

    if use2d:
        v1 = np.array([pose_1.x, pose_1.y])
        v2 = np.array([pose_2.x, pose_2.y])
    else:
        v1 = np.array([pose_1.x, pose_1.y, pose_1.z])
        v2 = np.array([pose_2.x, pose_2.y, pose_2.z])
    dist = np.linalg.norm(v2-v1)
    return dist



##################################################################################
#                          Функция поворота целевой точки
##################################################################################
def rotate_vect(angle, vec):
    """
    Поворачиваем b относительно a на угол rot
    :type vec: list
    :type angle: float

    :return: возвращаем точку повёрнутую на нужный угол

    """
    # матрица поворота
    rotate = np.array([[np.cos(angle), -np.sin(angle)],
                       [np.sin(angle), np.cos(angle)]])
    # разница координат дрона и ЦТ

    pos = np.array([[vec[0]], [vec[1]]])
    try:
        # повернутый вектор
        val = np.dot(rotate, pos)
        # normVec = val / np.linalg.norm(val) * dist
    except:
        val = [vec[0], vec[1]]
        print "rotate except"

    return [val[0][0],val[1][0],vec[2]]

def setup_market(pose, id):
    """
    Настройка маркера для отображения в rviz

    """
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "path"
    marker.id = id
    marker.action = 0
    marker.pose.orientation.x = pose.orientation.x
    marker.pose.orientation.y = pose.orientation.y
    marker.pose.orientation.z = pose.orientation.z
    marker.pose.orientation.w = pose.orientation.w

    marker.scale.x = 0.4
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.type = Marker.ARROW
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose.position.x = pose.position.x
    marker.pose.position.y = pose.position.y
    marker.pose.position.z = pose.position.z
    return marker


def getMarkerArray(path):
    markers = MarkerArray()
    id = 0
    for i in path.poses:
        markers.markers.append(setup_market(i,id))
        id+=1
    return markers
