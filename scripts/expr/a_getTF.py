#! /usr/bin/env python

import rospy
import geometry_msgs.msg

import numpy as np
import math

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path


pathTopic = "/aft_mapped_path"
saveName = "path.npy"

def callback(P: Path):
    keep = np.zeros([0, 3])
    for i in P.poses:
        pos = np.array([[i.pose.position.x, i.pose.position.y, i.pose.position.z]])
        keep = np.concatenate([keep, pos])
    np.save(saveName, keep)
    print(keep.shape)

def main():
    rospy.init_node('getTF')
    rate = rospy.Rate(10.0)
    sub = rospy.Subscriber(pathTopic, Path, callback)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
