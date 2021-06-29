#! /usr/bin/env python

import os
import natsort
import yaml
import pandas as pd
import math
from pypcd import pypcd

import rospy
import rospkg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.point_cloud2 import PointCloud2


os.chdir(rospkg.RosPack().get_path('odometry_ros'))
with open("./config/conf.yml", 'r') as f:
    conf = yaml.safe_load(f)
fileList = natsort.natsorted(os.listdir(conf['pcdDir']))
fileList = [os.path.join(conf['pcdDir'], f) for f in fileList]

def main():
    rospy.init_node("showPCD")
    rate = rospy.Rate(60)
    pub = rospy.Publisher("/velodyne_points", PointCloud2, queue_size=100)

    idx = 0
    while not rospy.is_shutdown():
        print("{0:05d}/{1:05d}".format(idx+1, len(fileList)), end="\r")

        thisPCD = pypcd.PointCloud.from_path(fileList[idx])
        pcdmsg = thisPCD.to_msg()
        pcdmsg.header.frame_id = 'pos'
        pcdmsg.header.stamp = rospy.Time()
        pub.publish(pcdmsg)

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
