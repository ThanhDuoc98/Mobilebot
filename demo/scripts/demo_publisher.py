#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# talker
def talker():
    pub = rospy.Publisher('hello_pub', String, queue_size=10)
    rospy.init_node('demo_publisher', anonymous=True)
    r = rospy.Rate(1) # 1hz
    count = 0
    while not rospy.is_shutdown():
        massage = str(count)
        rospy.loginfo(massage)
        pub.publish(massage)
        r.sleep()
        count += 1

# main
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass