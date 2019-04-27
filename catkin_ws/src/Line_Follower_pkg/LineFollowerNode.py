#!/usr/bin/python2.7
import rospy
#import for ros msg images
from sensor_msgs.msg import Image
#import for opencv:image operation, cv_bridge: convert msg to image, numpy: math operations
import cv2, cv_bridge, numpy
#import for sending out velocity messages (linear and angular)
from geometry_msgs.msg import Twist
class Follower:
	def __init__(self):
		#init bridge

		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		cv2.waitKey(3)
		#subscribe to image publisher node
		self.image_sub = rospy.Subscriber('/usb_cam/image_raw',Image, self.image_callback)
		#publish velocity messages
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/Line_Follower_vel',Twist, queue_size=1)
		self.twist = Twist()

	#callback function which runs when a message is recieved from subscribtion node (image node)
	def image_callback(self, msg):
		#convert message to image
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')

		#flip input
		h, w, c = image.shape

		h = h - 1
		w = w - 1

		empty_img = numpy.zeros([h, w, 3], dtype=numpy.uint8)

		for i in range(h):
			for j in range(w):
				empty_img[i, j] = image[h - i, w - j]
				empty_img = empty_img[0:h, 0:w]

		image = empty_img



		#convert image color scheme to HSV
		hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
		#define color of line in HSV
		lower_yellow = numpy.array([0, 145, 100])
		upper_yellow = numpy.array([5, 255, 255])
		#create binary mask by threshholding based on color
		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		#get image dimensions in pixels
		h, w, d = image.shape
		#set mask to black for everything not in ROI
		search_top = 3*h/4
		search_bot = search_top + 20
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0
		#some features of the mask (center,area,etc..)
		M = cv2.moments(mask)
		#when a line is detected publish rotation and constant velocity
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			#error is the center of the screen minus the center of the mask
			err = cx - w/2
			#set color to red or green depending on error value
			if(cx > w/2 - 20 and cx < w/2 + 20):
				color = (50,255,50)
			else:
				 color = (0,0,255)
				 print(err)
			cv2.circle(image, (w/2, cy), 20, color, 1)
			cv2.circle(image, (cx, cy), 5, color, -1)
			self.twist.linear.x = 0.05
			self.twist.angular.z = -float(err) / 100
			self.cmd_vel_pub.publish(self.twist)
		#when no line is detected publish rotation and velocity of 0
		else:
			print("no line detected")
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0
			self.cmd_vel_pub.publish(self.twist)

		cv2.imshow("window", image)
		cv2.waitKey(3)
#initialize this node
rospy.init_node('Line_Follower_Node')
#make new object of class
follower = Follower()
#start spinning without revisiting main fun
rospy.spin()
