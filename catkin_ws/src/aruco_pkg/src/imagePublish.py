#!/usr/bin/python2.7
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()


def show_webcam():
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        raise IOError("Cannot open webcam")

    while True:
        ret_val, img = cam.read()
        cv2.imshow("webcam", img)

        img_message = bridge.cv2_to_imgmsg(img, encoding="passthrough")
        pub.publish(img_message)

        key = cv2.waitKey(1)
        if key == 27:
            break

    cam.release()
    cv2.destroyAllWindows()


pub = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=1)
rospy.init_node("img_publisher", anonymous=True)

show_webcam()

'''rate = rospy.Rate(1)
while not rospy.is_shutdown():
    img = cv2.imread("/home/wout/Pictures/arucotest.jpg")
    img_message = bridge.cv2_to_imgmsg(img, encoding="passthrough")
    pub.publish(img_message)
    rate.sleep()
'''
