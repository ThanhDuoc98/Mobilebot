#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan


# split the laser scan into five distinct readings
def callback_laser(msg):
    # 640 / 5 = 128
    msg_array = []
    for i in range(640):
        if math.isnan(msg.ranges[i]):
            msg_array.append(5)
        else:
            msg_array.append(msg.ranges[i])

    regions = [
        min(min(msg_array[0:127]), 5),
        min(min(msg_array[128:255]), 5),
        min(min(msg_array[256:383]), 5),
        min(min(msg_array[384:511]), 5),
        min(min(msg_array[512:639]), 5)
    ]
    rospy.loginfo(regions)
        
    #rospy.loginfo(msg) # stdout, alike print


# reading laser
def laser_reading():
    rospy.init_node("reading_laser")
    sub = rospy.Subscriber("/scan", LaserScan, callback_laser)

    # keep node not sutdown
    rospy.spin()


# main
if __name__ == "__main__":
    laser_reading()
