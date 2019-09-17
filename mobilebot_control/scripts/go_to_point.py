#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math


# robot state variables
position_mobilebot = Point()
yaw_mobilebot = 0
# mobilebot state
state_mobielebot = 0
# goal
desired_position = Point()
desired_position.x = 0
desired_position.y = 0
desired_position.z = 0
# precision
yaw_precision = math.pi / 90 # +/- 2 degree
position_precision = 0.2

# publisher
pub = None


# change the state of motion planing
def change_state(state):
    global state_mobielebot
    state_mobielebot = state
    print("[Processing] State change to %s" %state_mobielebot)

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
        change_state(1)


# go straight to the desire point
def go_straight(desired_position):
    global yaw_mobilebot, yaw_precision, pub, state_mobielebot
    desired_yaw = math.atan2(desired_position.y - position_mobilebot.y, desired_position.x - position_mobilebot.x)
    yaw_error = desired_yaw - yaw_mobilebot
    position_error = math.sqrt(pow(desired_position.y - position_mobilebot.y, 2) + pow(desired_position.x - position_mobilebot.x, 2))
    
    if position_error > position_precision:
        twist_massage = Twist()
        twist_massage.linear.x = 1
        pub.publish(twist_massage)
    else:
        print("[INFO] Position error : %s" % position_error)
        change_state(2)
    
    # state change conditions
    if math.fabs(yaw_error) > yaw_precision:
        print("[INFO] Yaw error : %s" % yaw_error)
        change_state(0)

# task in the Odometry subcriber
def callback_odom(massage):
    global position_mobilebot
    global yaw_mobilebot

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

# the main function
def go_to_point():
    # create a pubisher and subcriber
    global pub
    rospy.init_node("go_to_point")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/odom", Odometry, callback_odom)

    rate = rospy.Rate(20) #20Hz

    while not rospy.is_shutdown():
        # state control mobilebot
        if state_mobielebot == 0:
            fix_yaw(desired_position)
        elif state_mobielebot == 1:
            go_straight(desired_position)
        elif state_mobielebot == 2:
            done()
            pass
        else:
            rospy.logerr('Unknown state!')
            pass

        rate.sleep()
    rate.sleep()


# run main function
if __name__ == "__main__":
    go_to_point()
