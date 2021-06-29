#! /usr/bin/env python2

import os
import yaml
import pandas as pd
import math

import tf
import rospy
import rospkg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


os.chdir(rospkg.RosPack().get_path('odometry_ros'))
with open("./config/conf.yml", 'r') as f:
    conf = yaml.safe_load(f)


def angleToOri(axis, theta):
    x = axis[0] * math.sin(theta/2.0)
    y = axis[1] * math.sin(theta/2.0)
    z = axis[2] * math.sin(theta/2.0)
    w = math.cos(theta/2.0)

    return (x, y, z, w)

def main():
    rospy.init_node("tfBr", anonymous=False)
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        # TODO:
        pos_x = 271005.44
        pos_y = 2745834.9
        pos_z = 301
        heading = 163
        heading = math.radians(-heading)

        br = tf.TransformBroadcaster()
        br.sendTransform(
            translation=(pos_x - conf['x_offset'], pos_y - conf['y_offset'], pos_z - conf['z_offset'] ),
            rotation=angleToOri([0, 0, 1], heading),
            time = rospy.Time(),
            child='pos',
            parent='world'
        )

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
