#!/usr/bin/env python3
"""
	Arduino reader/writer script
	Author : Mitchell Scott
		- misc4432@colorado.edu
	Project : Rufous
"""
import os
import sys
import yaml
import time
import rospy
import serial
import traceback
import numpy as np
import traceback as tb
from std_msgs.msg import String, Float64
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point, Quaternion


dType_LUT = {
	'0' : String,
	'1' : Float64,
	'2' : PoseStamped
}

class SerialAgent:
	def __init__(self, config):
		"""
		  A node that listens to an arduino sending one-liners.
		The first element of each msg is the id, the second is the 
		type of message. All messages are comma seperated sequences.
		There are a few optional parameters that can change 
		how you publish the data.
		"""
		self.nPorts = 0
		self.config = None
		self.device = None
		self.publishers = {}
		self.loadDevice(config)
		rospy.init_node('agent', anonymous=True)
		self.sub = rospy.Subscriber('/serial_out', String, self.writerCallback)
		self.initPublishers()

		self.tryConnect()

	def loadDevice(self, config):
		"""
		  Try to load the conf file for a device.

		"""
		kill = 0
		with open(config, 'r') as stream:
			self.config = yaml.safe_load(stream)

		self.log(self.config)
		if not 'State' in self.config:
			kill = 1
			self.log('Missing state description')
		elif not self.config['State']['WHOAMI']:
			self.config['State']['WHOAMI'] = 'Serial_Agent'
		if not 'Baudrate' in self.config:
			kill = 1
			self.log('Missing Baudrate')
		if not 'Ports' in self.config:
			kill = 1
			self.log('Missing Ports')
		else:
			 self.nPorts = len(self.config['Ports'])

		if kill:
			self.log('Check your config file')
			sys.exit(0)

	def tryConnect(self, alt=0):
		"""
		  Try to connect with the device.

		Returns:
		  status: bool - True if successful
		"""
		try:
			if self.device:
				self.device.close()

			self.device = serial.Serial(self.config['Ports'][alt], self.config['Baudrate'], timeout=self.config['Timeout'])
			self.device.flush()
			self.log('Device Connected: Reading...')
			self.connected = True

		except Exception as e:
			self.log(f'Error Detecting Device at {self.config["Ports"][alt]}')
			self.log(e)
			self.connected = False
			if self.nPorts > alt + 1:
				self.tryConnect(alt=alt+1)
		if not self.connected:
			time.sleep(5)

		self.writeBack(b'Z')
		return self.connected

	def initPublishers(self):
		"""
		  Creates publishers.
		"""
		for topic in self.config['State']:
			if topic == 'WHOAMI':
				continue
			dtype = dType_LUT[f'{self.config["State"][topic]}']
			self.publishers[topic] = rospy.Publisher(topic, dtype, queue_size=1)


	def writeBack(self, mesg):
		"""
		  Write a message back to the arduino
		"""
		self.device.write(mesg)

	def writerCallback(self, msg):
		"""
		  Callback for writing messages to the arduino
		"""
		self.writeBack(msg.data)

	def log(self, text):
		"""
		  Wrapper to print a value through /rosout.

			Params:
			  text: value - prints this
		"""
		rospy.loginfo(f'[Serial_Agent]: {text}')

	def parsePacket(self, packet, ts):
		"""
		  Create a new msg to publish.

			Params:
				topic: string - destination for the publisher
				msgType: string - the datatype of the msg
				data: ??? - the data to fill msg

			Returns:
			  msg: ??? - the msg to send
		"""
		msg = None
		for i,topic in enumerate(self.config['State']):

			if topic == 'WHOAMI':
				self.config['State']['WHOAMI'] = packet[i]	
				continue

			dt_id = self.config["State"][topic]
			dtype = dType_LUT[dt_id]

			if dt_id == '0':
				msg = String(packet[i])	

			if dt_id == '1':
				msg = Float64(float(packet[i]))

			if dt_id == '2':
				data = packet[i].split(',')
				msg = PoseStamped()
				msg.header.stamp = ts
				msg.header.frame_id = self.config['State']['WHOAMI']
				msg.pose.position = Point(float(data[0]), float(data[1]), float(data[2]))
				q = quaternion_from_euler(float(data[3]), float(data[4]), float(data[5]))
				msg.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

			if msg:
				self.publishers[topic].publish(msg)

	def spin(self):
		"""
		  Read a line from the serial port and publish it
		to a topic given the mode. The mode is the first 4 bytes
		of the msg.

			ROS Publishers:
			  - /***/string : String
			  - /***/float : Float
			  - /***/PoseStamped : PoseStamped
		"""
		try:
			while not rospy.is_shutdown():
				if not self.connected:
					self.tryConnect()

				elif self.device.in_waiting:
					packet = self.device.readline().decode().rstrip().split(':')
					self.parsePacket(packet, rospy.Time.now())

		except Exception as e:
			traceback.print_exc()
			self.log(e)
			self.log('Possible I/O Error: Restarting...')
			time.sleep(2)
			self.tryConnect()
			self.spin()

if __name__=='__main__':
	try:
		arduino_relay = SerialAgent(sys.argv[1])
		arduino_relay.log(f'{arduino_relay.config}')
		arduino_relay.spin()

	except KeyboardInterrupt:
		rospy.loginfo('[Serial_Agent]: Exiting ...')
		if arduio_relay.device:
			arduino_relay.device.close()

	except Exception as e:
		exc_type, exc_value, exc_traceback = sys.exc_info()
		tb.print_exc()
		if arduio_relay.device:
			arduino_relay.device.close()