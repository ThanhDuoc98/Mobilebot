#!/usr/bin/env python
#############################################################################
    # Project: Mobile Robot

    # File: SerialDataGateway.py
    
    # Description : Connect to STM32 via serial port

    # Output : Connection

    # Revision: 1.0

    # Status: IN_WORK

    # Date: 2019_09_26

    # Author: Tran Thanh Duoc _ DongThanh

    # Modify: 	# port
				# baudrate
#############################################################################
import threading
import serial
import time
import rospy

# print the line data received from serial port
def _OnLineReceived(line):
	print(line)


# Class SerialDataGateway
class SerialDataGateway(object):
	'''
	Helper class for receiving lines from a serial port
	'''

	def __init__(self, port="/dev/ttyACM1", baudrate=115200, lineHandler = _OnLineReceived):
		'''
		Initializes the receiver class. 
		port: The serial port to listen to.
		receivedLineHandler: The function to call when a line was received.
		'''
		self._Port = port
		self._Baudrate = baudrate
		self.ReceivedLineHandler = lineHandler
		self._KeepRunning = False


	def Start(self):
		'''
		Start serial
		'''
		self._Serial = serial.Serial(port = self._Port, baudrate = self._Baudrate, timeout = 1)
		self._KeepRunning = True
		self._ReceiverThread = threading.Thread(target=self._Listen)
		self._ReceiverThread.setDaemon(True)
		self._ReceiverThread.start()


	def Stop(self):
		'''
		Stop serial
		'''
		rospy.loginfo("[INFO]	Stopping serial gateway")
		self._KeepRunning = False
		time.sleep(.1)
		self._Serial.close()


	def _Listen(self):
		try:
			while self._KeepRunning:
				data = self._Serial.readline()
				self.ReceivedLineHandler(data)
		except:
			pass


	def Write(self, data):
		info = "[INFO]	Writing to serial port: %s" %data
		rospy.loginfo(info)
		self._Serial.write(str(data))


if __name__ == '__main__':
	dataReceiver = SerialDataGateway("/dev/ttyACM1",  115200)
	dataReceiver.Start()

	dataReceiver.Stop()
