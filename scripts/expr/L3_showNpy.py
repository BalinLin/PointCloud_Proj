#! /usr/bin/env python2

import os
import yaml
import math
import natsort
import numpy as np
import pandas as pd

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs import msg
from sensor_msgs.point_cloud2 import PointCloud2, PointField

import k_Data
import k_Util


kittiRoot = "/media/yoyo/harddisk/kitti_odmetry/2011_09_26/2011_09_26_drive_0001_sync/"
filename = "0000000000.npy"

def main():
    rospy.init_node("showNpy")
    rate = rospy.Rate(1)
    pub0 = rospy.Publisher("/KP", PointCloud2, queue_size=10)
    pub1 = rospy.Publisher("/KPNbr", PointCloud2, queue_size=10)
    pub2 = rospy.Publisher("/imuKP", PointCloud2, queue_size=10)
    pub3 = rospy.Publisher("/imuKPNbr", PointCloud2, queue_size=10)

    while not rospy.is_shutdown():
        npy = np.load(os.path.join(kittiRoot, "L3Data/KP", filename)
        print(npy.shape)
        pc2 = k_Util.xyzi_array_to_pointcloud2(npy, rospy.Time(), "world")
        pub0.publish(pc2)

        npy = np.load(os.path.join(kittiRoot, "L3Data/KPNbr", filename)
        npy = np.reshape(npy, (-1, 4))
        print(npy.shape)
        pc2 = k_Util.xyzi_array_to_pointcloud2(npy, rospy.Time(), "world")
        pub1.publish(pc2)

        npy = np.load(os.path.join(kittiRoot, "L3Data/IMUKP", filename)
        npy = np.reshape(npy, (-1, 4))
        print(npy.shape)
        pc2 = k_Util.xyzi_array_to_pointcloud2(npy, rospy.Time(), "world")
        pub2.publish(pc2)

        npy = np.load(os.path.join(kittiRoot, "L3Data/IMUKPNbr", filename)
        npy = npy[2, 2, 2, 0, ...]
        print(npy.shape)
        npy = np.reshape(npy, (-1, 4))
        print(npy.shape)
        pc2 = k_Util.xyzi_array_to_pointcloud2(npy, rospy.Time(), "world")
        pub3.publish(pc2)
        print("===")

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
