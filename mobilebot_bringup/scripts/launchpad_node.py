#!/usr/bin/env python
#############################################################################
    # Project: Mobile Robot

    # File: launchpad_node.py
    
    # Description : Receive sensor values from Launchpad board and publish as topics

    # Output : Topics

    # Revision: 1.0

    # Status: IN_WORK

    # Date: 2019_09_26

    # Author: Tran Thanh Duoc _ DongThanh

    # Modify:
#############################################################################
#Python client library for ROS
import rospy
import sys
import time
import math

#This module helps to receive values from serial port
from SerialDataGateway import SerialDataGateway
#Importing ROS data types
from std_msgs.msg import Int16, Int32, Int64, Float32, String, Header, UInt64

#Class to handle serial data from Launchpad and converted to ROS topics
class Launchpad_Class(object):
	
	def __init__(self):

		print("[INFO]	Initializing Launchpad Class")

		#Sensor variables
		self._Counter = 0

		self._left_encoder_value = 0
		self._right_encoder_value = 0

		self._left_wheel_speed_ = 0
		self._right_wheel_speed_ = 0

		self._LastUpdate_Microsec = 0
		self._Second_Since_Last_Update = 0

		self.robot_heading = 0

		#Get serial port and baud rate of STM32
		port = rospy.get_param("~port", "/dev/ttyACM1")
		baudRate = int(rospy.get_param("~baudRate", 115200))

		#Initializing SerialDataGateway with port, baudrate and callback function to handle serial data
		rospy.loginfo("[INFO]	Starting with serial port: " + port + ", baud rate: " + str(baudRate))
		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)
		rospy.loginfo("[INFO]	Started serial communication")
		

		#Subscribers and Publishers

		#Publisher for left and right wheel encoder values
		self._Left_Encoder = rospy.Publisher('lwheel', Int64, queue_size = 10)		
		self._Right_Encoder = rospy.Publisher('rwheel', Int64, queue_size = 10)		
	
		#Publisher for entire serial data
		self._SerialPublisher = rospy.Publisher('serial', String,queue_size=10)

		#Speed subscriber
		self._left_motor_speed = rospy.Subscriber('left_wheel_speed',Float32,self._Update_Left_Speed)
		self._right_motor_speed = rospy.Subscriber('right_wheel_speed',Float32,self._Update_Right_Speed)


	def _Update_Left_Speed(self, left_speed):

		self._left_wheel_speed_ = left_speed.data

		rospy.loginfo("[INFO]	" + str(self._left_wheel_speed_) + str(self._right_wheel_speed_))

		speed_message = 's %d %d\r' %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))

		self._WriteSerial(speed_message)
				

	def _Update_Right_Speed(self, right_speed):

		self._right_wheel_speed_ = right_speed.data

		rospy.loginfo("[INFO]	" + str(self._left_wheel_speed_) + str(self._right_wheel_speed_))

		speed_message = 's %d %d\r' %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))

		self._WriteSerial(speed_message)


	#publish left and right encoder value
	def _HandleReceivedLine(self,  line):
		self._Counter = self._Counter + 1
		self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))

		if(len(line) > 0):
			lineParts = line.split('\t')
			try:
				if(lineParts[0] == "a"):
					self._left_encoder_value = long(lineParts[1])
					self._right_encoder_value = long(lineParts[2])

					self._Left_Encoder.publish(self._left_encoder_value)
					self._Right_Encoder.publish(self._right_encoder_value)
				else:
					print("[ERROR]	" + line)
			except:
				rospy.logwarn("[ERROR]	Error in Sensor values")
				rospy.logwarn("[ERROR]	" + line)
				pass
			

	def _WriteSerial(self, message):
		self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
		self._SerialDataGateway.Write(message)

	def Start(self):
		rospy.logdebug("Starting")
		self._SerialDataGateway.Start()

	def Stop(self):
		rospy.logdebug("Stopping")
		self._SerialDataGateway.Stop()


	def Reset_Launchpad(self):
		print("\n[INFO]	Reset")
		reset = 'r\r'
		self._WriteSerial(reset)
		time.sleep(1)
		self._WriteSerial(reset)
		time.sleep(2)


if __name__ =='__main__':
	rospy.init_node('launchpad_ros',anonymous=True)
	launchpad = Launchpad_Class()
	try:
		
		launchpad.Start()	
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("[ERROR]	Error in main function")


	launchpad.Reset_Launchpad()
	launchpad.Stop()


