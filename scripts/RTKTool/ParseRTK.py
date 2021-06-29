#!/usr/bin/env python3
#coding=utf-8

import twd97
import pynmea2


def main():
    with open("/home/yoyo/0819_ncsist/ncsist1_rtk.txt", "w") as fw:
        with open("/home/yoyo/0819_ncsist/ncsist1_rtk_raw.txt", "r") as fr:
            lines = fr.readlines()
            ifgga = False
            for line in lines:
                msg = pynmea2.parse(line)
                if (msg.sentence_type == "GGA"):
                    if (msg.longitude > 121):
                        ifgga = True
                        twd97_x, twd97_y = twd97.fromwgs84(msg.latitude, msg.longitude)
                        fw.write("{0},{1},{2},{3},".format(msg.gps_qual, twd97_x, twd97_y, msg.altitude))
                if (msg.sentence_type == "VTG"):
                    ifgga = False
                    fw.write("{0}\n".format(msg.true_track))


if __name__ == "__main__":
    main()
