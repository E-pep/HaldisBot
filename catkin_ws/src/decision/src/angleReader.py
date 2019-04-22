#!/usr/bin/python2.7
import rospy
from std_msgs.msg import UInt16


def callback(data):
    print("jOEPIE!")
    print(data)


pub = rospy.Subscriber("/servo", UInt16, callback)
rospy.init_node("servo_listener", anonymous=True)

rospy.spin()
