#! /usr/bin/env python

import os
import time
import open3d
import numpy as np

from k_Util import EulerToRotationMatrix, RotationMatrixToEuler


def ReadPcd(filename):
    ''' Read PointCloud Map(format pcd) '''
    pcloud = open3d.io.read_point_cloud(filename)
    pcloud_np = np.asarray(pcloud.points)
    return pcloud_np

def ReadBin(filename):
    ''' Read velodyne_points(format bin) '''
    cloud = np.fromfile(filename, dtype=np.float32).reshape((-1, 4))
    return cloud

def ReadTimestamps(filename):
    with open(filename) as f:
        timesStr = [line for line in f]
        timestamps = []
        for i in timesStr:
            t_obj = time.strptime(i[:-11], "%Y-%m-%d %H:%M:%S")
            fixedSec = time.mktime(t_obj)
            mSeconds = float(i[-11:])
            timestamp = fixedSec + mSeconds
            timestamps += [timestamp]

        return timestamps

def Get_dTimes(timestamps):
    dTimes = []
    for i in range(len(timestamps)):
        if (i == 0):
            dTimes += [0]
        else:
            dTimes += [timestamps[i] - timestamps[i-1]]

    return dTimes

def ReadIMUDatas(filePath):
    imuDatas = []
    file_list = os.listdir(filePath)
    file_list.sort()
    for filename in file_list:
        with open(os.path.join(filePath, filename)) as f:
            line = f.readline()
            data = [float(i) for i in line.split(' ')]

            # [vF, vL, vU, vRool, vPitch, vYaw]
            imuData = [data[8], data[9], data[10], data[17], data[18], data[19]]
            imuDatas += [imuData]

    return np.array(imuDatas)

def Get_IMUPos(imuDatas, dtimes):
    imuPos = []
    for i in range(len(dtimes)):
        if (i == 0):
            imuPos += [np.array([0, 0, 0, 0, 0, 0])]
        else:
            vRotation = imuDatas[i-1, 3:6]
            rotation = imuPos[i-1][3:6] + vRotation * dtimes[i]

            RM = EulerToRotationMatrix([rotation[0], rotation[1], rotation[2]])
            vFLU = imuDatas[i-1, 0:3]
            vXYZ = np.array(vFLU).dot(RM)
            position = imuPos[i-1][0:3] + vXYZ * dtimes[i]

            imuPos += [np.concatenate([position, rotation], axis=-1)]

    return np.array(imuPos)

def ReadGTPos(poseTXT):
    with open(poseTXT) as f:
        poses = np.array([[float(x) for x in line.split()] for line in f])
        gtPosition = poses[:, 12:15]

        R = np.array([[[x[0], x[4], x[8]], [x[1], x[5], x[9]], [x[2], x[6], x[10]]] for x in poses])
        gtRotation = np.array([RotationMatrixToEuler(x) for x in R])

    return np.concatenate([gtPosition, gtRotation], axis=-1)

def Get_dPos(pos, dtimes):
    dPos = []
    for i in range(pos.shape[0] - 1):
        dPos += [(pos[i+1] - pos[i]) / dtimes[i+1]]

    return np.array(dPos)
