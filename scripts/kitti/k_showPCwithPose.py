#! /usr/bin/env python2

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs import msg
from sensor_msgs.point_cloud2 import PointCloud2, PointField

import os
import yaml
import math
import natsort
import numpy as np
import pandas as pd

import k_Data
import k_Util


kittiRoot = "/media/yoyo/harddisk/kitti_odmetry/2011_09_26/2011_09_26_drive_0001_sync/"

pcdRoot = os.path.join(kittiRoot, "velodyne_points/data/")
pcdPaths = [os.path.join(pcdRoot, x) for x in natsort.natsorted(os.listdir(pcdRoot))]

poseTXT = os.path.join(kittiRoot, "pose.txt")
gt = k_Data.ReadGTPos(poseTXT)

dataLens = len(pcdPaths)

def main():
    rospy.init_node("showPCwithPose")
    rate = rospy.Rate(10)
    br = tf.TransformBroadcaster()
    pub = rospy.Publisher("/velodyne_points", PointCloud2, queue_size=100)

    nowID = 0

    while not rospy.is_shutdown():
        br.sendTransform(
            translation=(gt[nowID, 0], gt[nowID, 1], gt[nowID, 2]),
            rotation=k_Util.AngleToOri([0, 0, 1], gt[nowID, 5]),
            time = rospy.Time(),
            child='pos',
            parent='world'
        )

        npArr = k_Data.ReadBin(pcdPaths[nowID])
        pc2 = k_Util.xyzi_array_to_pointcloud2(npArr, rospy.Time(), 'pos')
        pub.publish(pc2)

        nowID += 1
        if (nowID >= dataLens):
            nowID = 0
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
