#!/usr/bin/env python
import rospy
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Int64


class PublishOdom:

    def __init__(self):
        rospy.init_node("publish_odom")
        self.node_name = rospy.get_name()
        rospy.loginfo("- I %s started" % self.node_name)

        # parameters
        self.rate = 100000
        self.odom_frame_id = 'odom'
        self.base_frame_id = 'base_link'

        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def update(self):
        now = rospy.Time.now()
        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0
        quaternion.y = 0
        quaternion.z = 0
        quaternion.w = 0

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = 1
        odom.pose.pose.position.y = 1
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        # odom.child_frame_id = self.base_frame_id
        # odom.twist.twist.linear.x = self.dx
        # odom.twist.twist.linear.y = 0
        # odom.twist.twist.angular.z = self.dr
        self.odomPub.publish(odom)


if __name__ == "__main__":
    publishOdom = PublishOdom()
    publishOdom.spin()
