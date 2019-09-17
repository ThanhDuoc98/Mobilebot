#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan


# split the laser scan into five distinct readings
def callback_laser(msg):
    # 640 / 5 = 128
    regions = [
        min(min(msg.ranges[0:127]), 5),
        min(min(msg.ranges[128:255]), 5),
        min(min(msg.ranges[256:383]), 5),
        min(min(msg.ranges[384:511]), 5),
        min(min(msg.ranges[512:639]), 5)
    ]
    for i in range(5):
        if math.isnan(regions[i]):
            regions[i] = 5
        
    rospy.loginfo(msg) # stdout, alike print


# reading laser
def laser_reading():
    rospy.init_node("reading_laser")
    sub = rospy.Subscriber("/scan", LaserScan, callback_laser)

    # keep node not sutdown
    rospy.spin()


# main
if __name__ == "__main__":
    laser_reading()
