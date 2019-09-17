#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math
import time
from std_srvs.srv import *
from obstacle_avoidance import take_action


# service control mobilebot
# srv_client_go_to_point = None
# srv_client_obstacle_avoidance = None

# robot state variables
position_mobilebot = Point()
yaw_mobilebot = 0
# mobilebot state
state_mobielebot = 0
state_desciption = ["Go to point", "Obstacle avoidance"]
step_point = 0
# goal
desired_position = Point()
desired_position.x = 0
desired_position.y = 0
desired_position.z = 0
# precision
yaw_precision = math.pi / 90 # +/- 2 degree
position_precision = 0.2

pub = None
regions = None

# change state mobilebot
def change_state(state):
    global state_desciption, state_mobielebot
    # global srv_client_go_to_point, srv_client_obstacle_avoidance
    if state is not state_mobielebot:
        state_mobielebot = state
        print("[Process] State change to [%s] - %s" %(state, state_desciption[state]))
        # if state_mobielebot == 0:
        #     resp = srv_client_go_to_point(True)
        #     resp = srv_client_obstacle_avoidance(False)
        # if state_mobielebot == 1:
        #     resp = srv_client_go_to_point(False)
        #     resp = srv_client_obstacle_avoidance(True)

# return the position of mobilebot
def callbak_odom(massage):
    global position_mobilebot, yaw_mobilebot

    # take position of mobilebot
    position_mobilebot = massage.pose.pose.position

    # take yaw of mobilebot
    quaternion = (
        massage.pose.pose.orientation.x,
        massage.pose.pose.orientation.y,
        massage.pose.pose.orientation.z,
        massage.pose.pose.orientation.w,
    )
    (roll, pitch, yaw) = transformations.euler_from_quaternion(quaternion)
    yaw_mobilebot = yaw

# reading laser and split into five distinct
def callback_laser(msg):
    global regions
    # 650 / 5 = 130
    regions = {
        "right":  min(min(msg.ranges[0:129]), 10),
        "fright": min(min(msg.ranges[130:259]), 10),
        "front":  min(min(msg.ranges[260:389]), 10),
        "fleft":  min(min(msg.ranges[390:519]), 10),
        "left":   min(min(msg.ranges[520:649]), 10)
    }

# change the step of go to point
def change_step_point(step):
    global step_point
    step_point = step
    print("[Step] State change to %s" %step_point)

# finished the task, stop mobilebot
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

# fix the direction to desire point
def fix_yaw(desired_position):
    global yaw_mobilebot, yaw_precision, pub, state_mobielebot, position_mobilebot
    desired_yaw = math.atan2(desired_position.y - position_mobilebot.y, desired_position.x - position_mobilebot.x)
    yaw_error = desired_yaw - yaw_mobilebot

    # twist massage
    twist_massage = Twist()

    if math.fabs(yaw_error) > yaw_precision:
        twist_massage.angular.z = 0.5 if yaw_error > 0 else -0.7

    pub.publish(twist_massage)

    # check done and change state
    if math.fabs(yaw_error) <= yaw_precision:
        print("[INFO] Yaw error : %s" % yaw_error)
        change_step_point(1)


# go straight to the desire point
def go_straight(desired_position):
    global yaw_mobilebot, yaw_precision, pub, state_mobielebot
    desired_yaw = math.atan2(desired_position.y - position_mobilebot.y, desired_position.x - position_mobilebot.x)
    yaw_error = desired_yaw - yaw_mobilebot
    position_error = math.sqrt(pow(desired_position.y - position_mobilebot.y, 2) + pow(desired_position.x - position_mobilebot.x, 2))
    
    if position_error > position_precision:
        twist_massage = Twist()
        twist_massage.linear.x = 0.5
        pub.publish(twist_massage)
    else:
        print("[INFO] Position error : %s" % position_error)
        change_step_point(2)
    
    # state change conditions
    if math.fabs(yaw_error) > yaw_precision:
        print("[INFO] Yaw error : %s" % yaw_error)
        change_step_point(0)

# # control mobilrbot via data from laser
# def take_action(regions):
#     # massage to send subcriber
#     twist = Twist()
#     linear_x = 0
#     angular_z = 0

#     state_description = ""
#     distance_avoid = 0.5

#     if regions["front"] > distance_avoid and regions["fright"] > distance_avoid and regions["fleft"] > distance_avoid:
#         state_description = "case 1 : noting"
#         linear_x = 0.5
#         angular_z = 0
#     elif regions["front"] < distance_avoid and regions["fright"] > distance_avoid and regions["fleft"] > distance_avoid:
#         state_description = "case 2 : front"
#         linear_x = 0
#         angular_z = 0.5
#     elif regions["front"] > distance_avoid and regions["fright"] < distance_avoid and regions["fleft"] > distance_avoid:
#         state_description = "case 3 : fringht"
#         linear_x = 0
#         angular_z = 0.5
#     elif regions["front"] > distance_avoid and regions["fright"] > distance_avoid and regions["fleft"] < distance_avoid:
#         state_description = "case 4 : fleft"
#         linear_x = 0
#         angular_z = -0.5
#     elif regions["front"] < distance_avoid and regions["fright"] < distance_avoid and regions["fleft"] > distance_avoid:
#         state_description = "case 5 : front and fright"
#         linear_x = 0
#         angular_z = 0.5
#     elif regions["front"] < distance_avoid and regions["fright"] > distance_avoid and regions["fleft"] < distance_avoid:
#         state_description = "case 6 : front and fleft"
#         linear_x = 0
#         angular_z = -0.5
#     elif regions["front"] < distance_avoid and regions["fright"] < distance_avoid and regions["fleft"] < distance_avoid:
#         state_description = "case 7 : front and fright and fleft"
#         linear_x = 0
#         angular_z = 0.5
#     elif regions["front"] > distance_avoid and regions["fright"] < distance_avoid and regions["fleft"] < distance_avoid:
#         state_description = 'case 8 - fleft and fright'
#         linear_x = 0.3
#         angular_z = 0
#     else:
#         state_description = "not defined"

#     print(state_description)
#     twist.linear.x = linear_x
#     twist.angular.z = angular_z
#     pub.publish(twist)

# go to the point and obstacle avoidance
def go_point_obstacle():
    global pub
    global srv_client_go_to_point, srv_client_obstacle_avoidance
    global regions, position_mobilebot, desired_position, state_mobielebot, yaw_mobilebot, yaw_precision, position_precision
    rospy.init_node("go_point_obstacle")
    # Subcriber
    sub_laser = rospy.Subscriber("/laser", LaserScan, callback_laser)
    sub_odom = rospy.Subscriber("/odom", Odometry, callbak_odom)
    # Publisher
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # Service
    srv_client_go_to_point = rospy.ServiceProxy("/go_to_point_switch", SetBool)
    srv_client_obstacle_avoidance = rospy.ServiceProxy("/obstacle_avoidance_switch", SetBool)

    # initial going to the point
    change_state(0)
    
    rate = rospy.Rate(20) # 20Hz
    while not rospy.is_shutdown():
        if regions == None:
            continue

        if state_mobielebot == 0:
                    # state control mobilebot
            if step_point == 0:
                fix_yaw(desired_position)
            elif step_point == 1:
                go_straight(desired_position)
            elif step_point == 2:
                done()
                pass
            else:
                rospy.logerr('Unknown state!')
                pass
            if regions["front"] < 1 or regions["fright"] < 1 or regions["fleft"] < 1 or regions["right"] < 0.5 or regions["left"] < 0.5:
                change_state(1)
        else:
            take_action(regions)
            # desired_yaw = math.atan2(desired_position.y - position_mobilebot.y, desired_position.x - position_mobilebot.x)
            # error_yaw = desired_yaw - yaw_mobilebot
            if regions["front"] > 1 and regions["fright"] > 1 and regions["fleft"] > 1:
                change_state(0)
            
        rate.sleep()


# main
if __name__ == "__main__":
    go_point_obstacle()
