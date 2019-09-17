#!/usr/bin/env python

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
import time
import math

# control mobilebot
def control_mobilebot():
    # create a publisher
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard', anonymous=True)

    # set initial params
    speed = rospy.get_param("~speed", 1)
    turn = rospy.get_param("~turn", 1.8)
    x = 0
    y = 0
    z = 0
    th = 1
    status = 1

    # massage to send subcriber
    twist = Twist()
    # set time
    time_change = 4
    time_start = time.time()
    time_stop = time.time() + time_change
    count = 0

    rate = rospy.Rate(50000) # 1hz

    if status == 1:
        while not rospy.is_shutdown():
            if (time_start + time_change) >= time.time():
                turn = math.pi
                twist.angular.z = th*turn
                pub.publish(twist)
            else:
                # print(time.time() - time_start)
                turn = 0.0
                twist.angular.z = th*turn
                pub.publish(twist)
                if (time_stop + time_change) <= time.time():
                    time_start = time.time()
                    time_stop = time.time() + time_change
                    status = 0
                    count += 1
                    if count == 4:
                        break
            rate.sleep()

                # twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
                # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
                # pub.publish(twist)


# run
if __name__ == "__main__":
    control_mobilebot()