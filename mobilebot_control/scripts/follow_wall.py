#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math
import time


pub = None              # publisher
state_mobielebot = 0    # mobilebot state
state_dist = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

# change the state of motion planing
def change_state(state):
    global state_mobielebot, state_dist
    if state is not state_mobielebot:
        state_mobielebot = state
        print("[Processing] State change to [%s] - %s" %(state, state_dist[state]))

# detect and control
def take_action(regions):

    state_description = ""
    distance_avoid = 1

    if regions["front"] > distance_avoid and regions["fright"] > distance_avoid and regions["fleft"] > distance_avoid:
        state_description = "case 1 : noting"
        change_state(0)
    elif regions["front"] < distance_avoid and regions["fright"] > distance_avoid and regions["fleft"] > distance_avoid:
        state_description = "case 2 : front"
        change_state(1)
    elif regions["front"] > distance_avoid and regions["fright"] < distance_avoid and regions["fleft"] > distance_avoid:
        state_description = "case 3 : fringht"
        change_state(2)
    elif regions["front"] > distance_avoid and regions["fright"] > distance_avoid and regions["fleft"] < distance_avoid:
        state_description = "case 4 : fleft"
        change_state(0)
    elif regions["front"] < distance_avoid and regions["fright"] < distance_avoid and regions["fleft"] > distance_avoid:
        state_description = "case 5 : front and fright"
        change_state(1)
    elif regions["front"] < distance_avoid and regions["fright"] > distance_avoid and regions["fleft"] < distance_avoid:
        state_description = "case 6 : front and fleft"
        change_state(1)
    elif regions["front"] < distance_avoid and regions["fright"] < distance_avoid and regions["fleft"] < distance_avoid:
        state_description = "case 7 : front and fright and fleft"
        change_state(1)
    elif regions["front"] > distance_avoid and regions["fright"] < distance_avoid and regions["fleft"] < distance_avoid:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = "not defined"

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

# state 0
def find_wall():
    twist = Twist()
    twist.linear.x = 0.3
    twist.angular.z = -0.3
    return twist

# state 1
def turn_left():
    twist = Twist()
    twist.angular.z = 0.3
    return twist

# state 2
def following_wall():
    twist = Twist()
    twist.linear.x = 0.5
    return twist

def follow_wall():
    global pub

    rospy.init_node("follow_wall")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    sub = rospy.Subscriber("/laser", LaserScan, callback_laser)

    rate = rospy.Rate(20) #20Hz
    while not rospy.is_shutdown():
        twist = Twist()
        if state_mobielebot == 0:
            twist = find_wall()
        elif state_mobielebot == 1:
            twist = turn_left()
        elif state_mobielebot == 2:
            twist = following_wall()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub.publish(twist)

        rate.sleep()


# main
if __name__ == "__main__":
    follow_wall()
