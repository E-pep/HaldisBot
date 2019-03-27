#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge, numpy
from geometry_msgs.msg import Twist
class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		cv2.waitKey(3)
		self.image_sub = rospy.Subscriber('/usb_cam/image_raw',Image, self.image_callback) #/usb_cam/image_raw/Image
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/Line_Follower_vel',Twist, queue_size=1)
		self.twist = Twist()
	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_yellow = numpy.array([20, 100, 100])
		upper_yellow = numpy.array([30, 255, 255])
		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		h, w, d = image.shape
		search_top = 3*h/4
		search_bot = search_top + 20
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0
		M = cv2.moments(mask)
		#when a line is detected publish rotation and constant velocity
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			err = cx - w/2
			if(cx > w/2 - 20 and cx < w/2 + 20):
				color = (50,255,50)
			else:
				 color = (0,0,255)
				 print(err)
			cv2.circle(image, (w/2, cy), 20, color, 1)#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge, numpy
from geometry_msgs.msg import Twist
class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		cv2.waitKey(3)
		self.image_sub = rospy.Subscriber('/usb_cam/image_raw',Image, self.image_callback) #/usb_cam/image_raw/Image
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/Line_Follower_vel',Twist, queue_size=1)
		self.twist = Twist()
	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_yellow = numpy.array([20, 100, 100])
		upper_yellow = numpy.array([30, 255, 255])
		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		h, w, d = image.shape
		search_top = 3*h/4
		search_bot = search_top + 20
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0
		M = cv2.moments(mask)
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			err = cx - w/2
			if(cx > w/2 - 20 and cx < w/2 + 20):
				color = (50,255,50)
				err = 0;
			else:
				 color = (0,0,255)
			cv2.circle(image, (w/2, cy), 20, color, 1)
			cv2.circle(image, (cx, cy), 5, color, -1)
			self.twist.linear.x = 0.2
			self.twist.angular.z = -float(err) / 100
			self.cmd_vel_pub.publish(self.twist)
			print("Velocity msg published {}".format(err))
		else:
			print("NO Velocity msg published")
		cv2.imshow("window", image)
		cv2.waitKey(3)

rospy.init_node('Line_Follower_Node')
follower = Follower()
rospy.spin()

			cv2.circle(image, (cx, cy), 5, color, -1)
			self.twist.linear.x = 0.2
			self.twist.angular.z = -float(err) / 100
			self.cmd_vel_pub.publish(self.twist)
		#when no line is detected publish rotation and velocity of 0
		else:
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0
			self.cmd_vel_pub.publish(self.twist)

		cv2.imshow("window", image)
		cv2.waitKey(3)

rospy.init_node('Line_Follower_Node')
follower = Follower()
rospy.spin()
