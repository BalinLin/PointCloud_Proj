#!/usr/bin/env python3
#coding=utf-8

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Quaternion, Point, PoseStamped
from tf.transformations import quaternion_from_euler

import math
import twd97
import pynmea2


def talker():
    rospy.init_node("rtktxt_pather")
    pub = rospy.Publisher("/rtkPath", Path, queue_size=1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rtkPath = Path()
        rtkPath.header.frame_id = "/twd97"
        rtkPath.header.stamp = rospy.Time()

        with open("/home/yoyo/0819_ncsist/ncsist1_rtk.txt", "r") as f:
            lines = f.readlines()
            for l in lines:
                r_qual, r_twd97_x, r_twd97_y, r_height, r_dir = [float(i) for i in l.split(',')]
                p = PoseStamped()
                p.pose.position = Point(r_twd97_x, r_twd97_y, r_height)
                p.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.radians((-r_dir+90))).tolist())
                rtkPath.poses.append(p)

            pub.publish(rtkPath)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInternalException:
        pass
