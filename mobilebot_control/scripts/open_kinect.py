#!/usr/bin/env python

import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import numpy as np
import math

class cvBridgeDemo():
    def __init__(self):
        self.node_name = "cv_bridge_demo"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.qrDecoder = cv2.QRCodeDetector()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback_laser)
        rospy.Timer(rospy.Duration(0.03), self.show_img_cb)
        rospy.loginfo("Waiting for image topics...")

    def show_img_cb(self,event):
        try:
            cv2.namedWindow("RGB_Image", cv2.WINDOW_NORMAL)
            cv2.moveWindow("RGB_Image", 25, 75)
            cv2.imshow("RGB_Image",self.frame)
            cv2.waitKey(3)
        except:
            pass

    def image_callback(self, ros_image):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
            pass
        frame = np.array(self.frame, dtype=np.uint8)
        data,bbox,rectifiedImage = self.qrDecoder.detectAndDecode(frame)
        if len(data)>0:
            print("Decoded Data : {}".format(data))

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

    def callback_laser(self, msg):
        # 640 / 5 = 128
        msg_array = []
        for i in range(640):
            if math.isnan(msg.ranges[i]):
                msg_array.append(5)
            else:
                msg_array.append(msg.ranges[i])
        
        a = msg_array[319]
        b = msg_array[630]
        c = msg_array[9]

        print str(a) + "    " + str(b) + "    " + str(c)

        if b < 1 and c < 1 and a < 1.2 and c < a and b < a:
            self.near_conner = True
            d = math.sqrt(pow(a, 2) + pow(b, 2) - 2*a*b*math.sin((math.pi/180)*28.5))
            beta = math.acos(math.sqrt(pow(a, 2) + pow(d, 2) - pow(b, 2))/(2*a*d))
            y = a * math.sin(beta)
            theta = a * math.cos(y/a)
            x = c * math.cos(math.pi/2 - theta - (math.pi/180)*28.5)
            print "x : " + str(x) + "    " + "y : " + str(y) + "  " + "theta : " + str(theta)
        else:
            pass
        

def main(args):
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)