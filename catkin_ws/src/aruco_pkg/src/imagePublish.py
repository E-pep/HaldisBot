#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

pub = rospy.Publisher("img_pub", Image, queue_size=1)
rospy.init_node("img_publisher", anonymous=True)
bridge = CvBridge()

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    img = cv2.imread("/home/wout/Pictures/arucotest.jpg")
    img_message = bridge.cv2_to_imgmsg(img, encoding="passthrough")
    pub.publish(img_message)
    rate.sleep()
