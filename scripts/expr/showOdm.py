#! /usr/bin/env python

import os
import yaml
import pandas as pd
import math
import numpy as np

import rospy
import rospkg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path


np.set_printoptions(precision=4, suppress=True)
os.chdir(rospkg.RosPack().get_path('odometry_ros'))
with open("./config/conf.yml", 'r') as f:
    conf = yaml.safe_load(f)
# prdFile = "/home/yoyo/catkin_ws/path.npy"

def main():
    rospy.init_node("showOdm")
    rate = rospy.Rate(10)
    gpsPathPub = rospy.Publisher("/gpsPath", Path, queue_size=100)
    imuPathPub = rospy.Publisher("/imuPath", Path, queue_size=100)
    prdPathPub = rospy.Publisher("/prdPath", Path, queue_size=100)

    while not rospy.is_shutdown():
        # GPS
        gpsPath = Path()
        gpsPath.header.frame_id = '/world'
        gpsPath.header.stamp = rospy.Time()

        # IMU
        imuPath = Path()
        imuPath.header.frame_id = '/world'
        imuPath.header.stamp = rospy.Time()

        df = pd.read_csv(conf['readCSV'])

        lastT = df.iloc[0]['UTC time']
        lastX = df.iloc[0]['X']
        lastY = df.iloc[0]['Y']
        lastZ = df.iloc[0]['Z']

        for i in df.axes[0]:
            # GPS
            p = PoseStamped()
            p.pose.position = Point(df.iloc[i, 1]-conf['x_offset'], df.iloc[i, 2]-conf['y_offset'], df.iloc[i, 3]-conf['z_offset'])
            gpsPath.poses.append(p)

            # line_imu
            if i > 0:
                speed = df.loc[i]['Speed']
                yawRt = df.loc[i]['YawRate']
                heading = df.loc[i]['Heading']

                dt = df.iloc[i]['UTC time'] - lastT
                dx = speed * math.cos(math.radians(-heading+90)) * dt
                dy = speed * math.sin(math.radians(-heading+90)) * dt

                lastT = df.iloc[i]['UTC time']
                lastX += dx
                lastY += dy
                lastZ = lastZ

                p = PoseStamped()
                p.pose.position = Point(lastX-conf['x_offset'], lastY-conf['y_offset'], lastZ-conf['z_offset'])
                imuPath.poses.append(p)

        gpsPathPub.publish(gpsPath)
        imuPathPub.publish(imuPath)
        print("GPS, IMU Path Published.")

        # Predict
        if "prdFile" in globals():
            prdPath = Path()
            prdPath.header.frame_id = '/world'
            prdPath.header.stamp = rospy.Time()

            theta = 4.2
            m = np.array(
                [
                    [math.cos(theta), -math.sin(theta), 0],
                    [math.sin(theta), math.cos(theta), 0],
                    [0, 0, 1]
                ])
            prdPose = np.load(prdFile)
            prdPose = np.dot(prdPose, m)
            prdPose += np.array([df.iloc[0]['X']-conf['x_offset'], df.iloc[0]['Y']-conf['y_offset'], df.iloc[0]['Z']-conf['z_offset']])

            for i in prdPose:
                p = PoseStamped()
                p.pose.position = Point(i[0], i[1], i[2])
                prdPath.poses.append(p)
            prdPathPub.publish(prdPath)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
