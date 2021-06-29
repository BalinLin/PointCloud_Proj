#!/usr/bin/env python3
#coding=utf-8

import rospy
import pynmea2
from std_msgs.msg import String


def callback(str_msg: String):
    segstr = str_msg.data.split(",")
    if segstr[0] == "$GPGGA" or segstr[0] == "$GNVTG":
        print(repr(str_msg.data))

        with open("/home/yoyo/0819_ncsist/ncsist1_rtk_raw.txt", "a+") as f:
            f.write(str_msg.data)


def listener():
    rospy.init_node("rtktxt_writer")
    sub = rospy.Subscriber("/rtk_str", String, callback)
    rate = rospy.Rate(100)

    rospy.spin()


if __name__ == "__main__":
    listener()
