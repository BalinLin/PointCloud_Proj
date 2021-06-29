#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path

import os
import yaml
import pandas as pd
import math
import numpy as np

import k_Data
import k_Util

kittiRoot = "/media/yoyo/harddisk/kitti_odmetry/2011_09_26/2011_09_26_drive_0005_sync/"

poseTXT = os.path.join(kittiRoot, "pose.txt")
gt = k_Data.ReadGTPos(poseTXT)

timePath = os.path.join(kittiRoot, "oxts/timestamps.txt")
IMUPath = os.path.join(kittiRoot, "oxts/data/")
times = k_Data.ReadTimestamps(timePath)
dtimes = k_Data.Get_dTimes(times)
imuDatas = k_Data.ReadIMUDatas(IMUPath)
imuPos = k_Data.Get_IMUPos(imuDatas, dtimes)

def main():
    rospy.init_node("showOdm")
    rate = rospy.Rate(10)
    gpsPathPub = rospy.Publisher("/path_GPS", Path, queue_size=100)
    imuPathPub = rospy.Publisher("/path_IMU", Path, queue_size=100)

    while not rospy.is_shutdown():
        # GPS
        gpsPath = Path()
        gpsPath.header.frame_id = '/camera_init'
        gpsPath.header.stamp = rospy.Time()
        for i in gt:
            p = PoseStamped()
            p.pose.position = Point(i[0], i[1], i[2])
            gpsPath.poses.append(p)
        gpsPathPub.publish(gpsPath)

        # IMU
        imuPath = Path()
        imuPath.header.frame_id = '/camera_init'
        imuPath.header.stamp = rospy.Time()
        for i in imuPos:
            p = PoseStamped()
            p.pose.position = Point(i[0], i[1], i[2])
            imuPath.poses.append(p)
        imuPathPub.publish(imuPath)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
