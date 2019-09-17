#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# control mobilrbot via data from laser
def take_action(regions):
    # massage to send subcriber
    twist = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ""
    distance_avoid = 0.5

    if regions["front"] > distance_avoid and regions["fright"] > distance_avoid and regions["fleft"] > distance_avoid:
        state_description = "case 1 : noting"
        linear_x = 1
        angular_z = 0
    elif regions["front"] < distance_avoid and regions["fright"] > distance_avoid and regions["fleft"] > distance_avoid:
        state_description = "case 2 : front"
        linear_x = 0
        angular_z = 0.5
    elif regions["front"] > distance_avoid and regions["fright"] < distance_avoid and regions["fleft"] > distance_avoid:
        state_description = "case 3 : fringht"
        linear_x = 0
        angular_z = 0.5
    elif regions["front"] > distance_avoid and regions["fright"] > distance_avoid and regions["fleft"] < distance_avoid:
        state_description = "case 4 : fleft"
        linear_x = 0
        angular_z = -0.5
    elif regions["front"] < distance_avoid and regions["fright"] < distance_avoid and regions["fleft"] > distance_avoid:
        state_description = "case 5 : front and fright"
        linear_x = 0
        angular_z = 0.5
    elif regions["front"] < distance_avoid and regions["fright"] > distance_avoid and regions["fleft"] < distance_avoid:
        state_description = "case 6 : front and fleft"
        linear_x = 0
        angular_z = -0.5
    elif regions["front"] < distance_avoid and regions["fright"] < distance_avoid and regions["fleft"] < distance_avoid:
        state_description = "case 7 : front and fright and fleft"
        linear_x = 0
        angular_z = 0.5
    elif regions["front"] > distance_avoid and regions["fright"] < distance_avoid and regions["fleft"] < distance_avoid:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0.3
        angular_z = 0
    else:
        state_description = "not defined"

    print(state_description)
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    pub.publish(twist)


# split the laser scan into five distinct readings
def callback_laser(msg):
    # 650 / 5 = 130
    regions = {
        "right":  min(min(msg.ranges[0:129]), 10),
        "fright": min(min(msg.ranges[130:259]), 10),
        "front":  min(min(msg.ranges[260:389]), 10),
        "fleft":  min(min(msg.ranges[390:519]), 10),
        "left":   min(min(msg.ranges[520:649]), 10)
    }
    # control mobilebot
    take_action(regions)


# reading laser and send Twist massage control mobilebot
def obstacle_avoidance():

    global pub
    rospy.init_node("reading_laser")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/laser", LaserScan, callback_laser)

    # keep node not sutdown
    rospy.spin()


# main
if __name__ == "__main__":
    obstacle_avoidance()
