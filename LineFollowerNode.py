#!/usr/bin/env python2.7
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge, numpy
class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		cv2.waitKey(3)
		self.image_sub = rospy.Subscriber('/image_publisher_1552472555706544959/image_raw',Image, self.image_callback) #/usb_cam/image_raw/Image
	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		h, w, d = image.shape
		M = cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
		dst = cv2.warpAffine(image,M,(cols,rows))		
		hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
		lower_yellow = numpy.array([ 10, 0, 0])
		upper_yellow = numpy.array([255, 255, 255])
		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)		
		search_top = 3*h/4
		search_bot = search_top + 20
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0
		M = cv2.moments(mask)
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(image, (w/2, h/2), 20, (0,0,255), 1)
			cv2.circle(image, (cx, cy), 5, (0,0,255), -1)
			err = cx - w/2
			self.twist.linear.x = 0.2
			self.twist.angular.z = -float(err) / 100
			self.cmd_vel_pub.publish(self.twist)
			cv2.imshow("window", image)
			cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
