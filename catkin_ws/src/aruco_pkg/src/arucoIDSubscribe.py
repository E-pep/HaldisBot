#!/usr/bin/python2.7
import rospy
from std_msgs.msg import Float32MultiArray


def callback(data):
    print(data.data[0], data.data[1])


pub = rospy.Subscriber("/id_pub", Float32MultiArray, callback)
rospy.init_node("id_listener", anonymous=True)

rospy.spin()
