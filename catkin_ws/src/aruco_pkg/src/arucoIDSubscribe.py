#!/usr/bin/python3
import rospy
from std_msgs.msg import Int32


def callback(data):
    print(data)


pub = rospy.Subscriber("id_pub", Int32, callback)
rospy.init_node("id_listener", anonymous=True)

rospy.spin()
