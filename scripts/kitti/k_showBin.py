#! /usr/bin/env python

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


binDir = "/media/yoyo/harddisk/kitti_odmetry/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data"
fileList = natsort.natsorted(os.listdir(binDir))
fileList = [os.path.join(binDir, f) for f in fileList]

def main():
    rospy.init_node("showBin")
    rate = rospy.Rate(5)
    pub = rospy.Publisher("/velodyne_points", PointCloud2, queue_size=100)

    idx = 0
    while not rospy.is_shutdown():
        print("{0:05d}/{1:05d}".format(idx+1, len(fileList)), end="\r")

        b = k_Data.ReadBin(fileList[idx])
        pc2 = k_Util.xyzi_array_to_pointcloud2(b, rospy.Time(), "velodyne")
        pub.publish(pc2)

        idx +=1
        if idx >= len(fileList):
            idx=0
            break
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
