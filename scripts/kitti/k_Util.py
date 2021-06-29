#! /usr/bin/env python

import os
import yaml
import math
import numpy as np
import pandas as pd

from sensor_msgs import msg
from sensor_msgs.point_cloud2 import PointCloud2, PointField


def xyzi_array_to_pointcloud2(points, stamp=None, frame_id=None):
    msg = PointCloud2()

    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id

    msg.height = 1
    msg.width = len(points)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)
        ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = np.asarray(points, np.float32).tostring()

    return msg

def EulerToRotationMatrix(theta):
    R_x = np.array([[1,                   0,                   0],
                    [0,  math.cos(theta[0]),  math.sin(theta[0])],
                    [0, -math.sin(theta[0]),  math.cos(theta[0])]])

    R_y = np.array([[ math.cos(theta[1]), 0, math.sin(theta[1])],
                    [                  0, 1,                  0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]])

    R_z = np.array([[ math.cos(theta[2]), math.sin(theta[2]), 0],
                    [-math.sin(theta[2]), math.cos(theta[2]), 0],
                    [                  0,                  0, 1]])

    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

def RotationMatrixToEuler(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    # [x, y, z] = [roll, pitch, yaw]
    return np.array([x, y, z], dtype=np.float32)

def PCTransform(pc, offset, theta):
    pc_xyz = pc[:, 0:3]
    pc_ins = pc[:, 3:4]
    R = EulerToRotationMatrix(theta)
    pc_T = pc_xyz.dot(R) + offset
    pc_T = np.hstack((pc_T, pc_ins))
    return pc_T

def AngleToOri(axis, theta):
    x = axis[0] * math.sin(theta/2.0)
    y = axis[1] * math.sin(theta/2.0)
    z = axis[2] * math.sin(theta/2.0)
    w = math.cos(theta/2.0)

    return (x, y, z, w)
