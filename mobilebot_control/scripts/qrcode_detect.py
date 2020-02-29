#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo, LaserScan
import cv2
import cv_bridge
import sys
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import numpy as np
import math


class QRCodeDetect:

    def __init__(self):
        self.near_conner = False
        self.bridge = cv_bridge.CvBridge()
        self.qrDecoder = cv2.QRCodeDetector()
        self.node_name = "qrcode_detect"
        rospy.init_node(self.node_name)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback_laser)
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber(
            '/camera/rgb/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        if self.near_conner:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            data, bbox, rectifiedImage = self.qrDecoder.detectAndDecode(image)
            if len(data) > 0:
                print("Decoded Data : {}".format(data))
                print(type(data))
                self.display(image, bbox)
            cv2.imshow('image', image)
        else:
            rospy.loginfo("Not near coner")

    def display(image, bbox):
        n = len(bbox)
        for j in range(n):
            cv2.line(image, tuple(bbox[j][0]), tuple(
                bbox[(j+1) % n][0]), (255, 0, 0), 3)

    # split the laser scan into five distinct readings
    def callback_laser(self, msg):
        # 640 / 5 = 128
        msg_array = []
        for i in range(640):
            if math.isnan(msg.ranges[i]):
                msg_array.append(5)
            else:
                msg_array.append(msg.ranges[i])

        if msg_array[0] < 1:
            self.near_conner = True
        else:
            self.near_conner = False

        # regions = [
        #     min(min(msg_array[0:127]), 5),
        #     min(min(msg_array[128:255]), 5),
        #     min(min(msg_array[256:383]), 5),
        #     min(min(msg_array[384:511]), 5),
        #     min(min(msg_array[512:639]), 5)
        # ]
        # rospy.loginfo(regions)


def main(args):
    try:
        QRCodeDetect()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
