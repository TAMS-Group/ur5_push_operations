#!/usr/bin/env python

from geometry_msgs.msg import Pose, Quaternion
from pyquaternion import Quaternion as PyQuaternion
import numpy as np

class Push(object):
    __slots__ = ['id', 'approach', 'angle', 'distance']

def get_diff_pose(p1, p2):
    t1 = pose_to_matrix(p1)
    t2 = pose_to_matrix(p2)
    t3 = np.dot(np.linalg.inv(t1), t2)
    return matrix_to_pose(t3)

def get_diff_pose_old(p1, p2):
    pose = Pose()
    pose.position.x = p2.position.x - p1.position.x
    pose.position.y = p2.position.y - p1.position.y
    pose.position.z = p2.position.z - p1.position.z
    pose.orientation = quat_difference(p1.orientation, p2.orientation)
    return pose

def pose_to_matrix(pose):
    T = quat_geom_to_py(pose.orientation).transformation_matrix
    T[0][-1] = pose.position.x
    T[1][-1] = pose.position.y
    T[2][-1] = pose.position.z
    return T

def matrix_to_pose(mat):
    pose = Pose()
    pose.position.x = mat[0][-1]
    pose.position.y = mat[1][-1]
    pose.position.z = mat[2][-1]
    pose.orientation = quat_py_to_geom(PyQuaternion(matrix=mat))
    return pose

def quat_slerp(q1, q2):
    if isinstance(q1, PyQuaternion) and isinstance(q2, PyQuaternion):
        return PyQuaternion.slerp(q1, q2)
    elif isinstance(q1, Quaternion) and isinstance(q2, Quaternion):
        q3 = PyQuaternion.slerp(quat_geom_to_py(q1), quat_geom_to_py(q2))
        return quat_py_to_geom(q3)
    else:
        return None

def quat_geom_to_py(geomquat):
    return PyQuaternion(w=geomquat.w, x=geomquat.x, y=geomquat.y, z=geomquat.z)

def quat_py_to_geom(pyquat):
    q = Quaternion()
    q.w = pyquat.elements[0]
    q.x = pyquat.elements[1]    
    q.y = pyquat.elements[2]
    q.z = pyquat.elements[3]
    return q

def quat_difference(q1, q2):
    if isinstance(q1, PyQuaternion) and isinstance(q2, PyQuaternion):
        return q1 / q2
    elif isinstance(q1, Quaternion) and isinstance(q2, Quaternion):
        q3 = quat_geom_to_py(q1) / quat_geom_to_py(q2)
        return quat_py_to_geom(q3)

def quat_from_yaw(yaw):
    return quat_py_to_geom(PyQuaternion(axis=[0,0,1], angle=yaw))

def get_yaw(pose):
    q = quat_geom_to_py(pose.orientation)
    return q.angle * q.axis[2]
