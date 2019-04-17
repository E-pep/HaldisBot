#!/usr/bin/python2.7

import rospy
from std_msgs.msg import Float32MultiArray, UInt16
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from numpy import pi
import time
import sys
import signal


def signal_handler(signal, frame):
    print("Program interrupted and exited")
    sys.exit(0)


"""
States
======
can be found at https:#www.draw.io/#G17kVh2GfapUA7j_Pc61OxIDDBFxVxKaeO
(Third tab from bottom)
"""


class DecisionNode:

    def __init__(self):
        self.sub = rospy.Subscriber("dummy", Image, self.dummy_func)
        rospy.init_node("decision_node", anonymous=True)
        self.margin = 0.05
        self.vel_com = Twist()
        self.aruco_threshold = 4
        self.detected_aruco = Float32MultiArray()
        self.aruco_to_find = 1000000
        self.state = 1
        self.previous_state = 0
        self.time_last = 0
        # TODO: test gripper angles!
        self.gripper_time = 3
        self.takeaway_time = 10
        self.gripper_open = UInt16(0)
        self.gripper_close = UInt16(80)

        self.gripper_angle = self.gripper_open
        self.pub = rospy.Publisher("/servo", UInt16, queue_size=3)
        self.pub.publish(self.gripper_angle)
        self.pub = rospy.Publisher("/servo", UInt16, queue_size=3)
        self.pub.publish(self.gripper_angle)

    def dummy_func(self):
        pass

    def turn_amount(self, degrees):
        time.sleep(1)
        self.vel_com.angular.z = 30 * 2 * pi / 360
        self.vel_com.linear.x = 0
        self.pub.publish(self.vel_com)
        time.sleep(degrees / 30)
        self.vel_com.angular.z = 0
        self.vel_com.linear.x = 0
        self.pub.publish(self.vel_com)

    def drive_time(self, speed, seconds):
        time.sleep(1)
        self.vel_com.linear.x = speed
        self.vel_com.angular.z = 0
        self.pub.publish(self.vel_com)
        time.sleep(seconds)
        self.vel_com.linear.x = 0
        self.vel_com.angular.z = 0
        self.pub.publish(self.vel_com)

    def aruco_instruction_callback(self, msg):
        self.detected_aruco = msg.data[0]
        if self.detected_aruco < self.aruco_threshold:
            self.aruco_to_find = self.detected_aruco
            print("Detected:", self.detected_aruco)
            self.state += 1

    def movement_callback(self, msg):
        # when no line is detected
        if msg.linear.x == 0:
            msg.angular.z = 0
            self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.pub.publish(msg)
            print("No line for:", time.time() - self.time_last, "Seconds")
            if time.time() - self.time_last > 5:
                self.state += 1

        # publish to topic to turtlebot when a line is detected
        else:
            self.time_last = time.time()  # set clock
            self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.pub.publish(msg)
            # reset the counter, still moving

    def aruco_drive_callback(self, msg):
        # extract data from aruco node
        self.detected_aruco = msg.data[0]
        xpercentage = msg.data[1]
        if self.detected_aruco == self.aruco_to_find:
            print("Found:", self.detected_aruco)
            # check whether aruco is in the middle (with a margin))
            if 0.5 - self.margin < xpercentage < 0.5 + self.margin:
                print("In middle!")
                # Stop turning and go to next state
                self.vel_com.linear.x = 0
                self.vel_com.angular.z = 0
                self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
                self.pub.publish(self.vel_com)
                self.state += 1
            # Drive up to the can

    def main(self):
        print("Decision node started")
        # Margin for center of ArUco detection state 3
        while True:
            if self.state != self.previous_state:
                self.previous_state = self.state

                # Scanning for ArUco instruction
                if self.state == 1:
                    print("State 1: Scanning for ArUco")
                    self.sub.unregister()
                    # subscribing to ArUco ID topic and use callback
                    self.sub = rospy.Subscriber("/id_pub", Float32MultiArray, self.aruco_instruction_callback)

                # Move to end of line
                elif self.state == 2:
                    print("State 2: Move to end of line")
                    # unsubscribing topic aruco
                    self.sub.unregister()
                    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
                    print("Turning 180")
                    self.turn_amount(180)
                    print("Start driving")
                    self.time_last = time.time()
                    self.sub = rospy.Subscriber("cmd_vel_mux/Line_Follower_vel", Twist, self.movement_callback)
                    # Turn around until something gets published on the node we're subscribed to
                    # Meaning we turn 180 degrees until we see the yellow line, then go into the callback

                # Rotate while centering for correct ArUco code
                elif self.state == 3:
                    print("State 3: Scanning for correct ArUco")
                    self.sub.unregister()
                    self.sub = rospy.Subscriber("/id_pub", Float32MultiArray, self.aruco_drive_callback)
                    # start turning (aruco_drive_callback will stop the turning)
                    self.vel_com.angular.z = 15 * 2 * pi / 360
                    self.vel_com.linear.x = 0
                    self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
                    self.pub.publish(self.vel_com)

                # Stop rotation, rotate 180 and drive backwards, then grip the object
                elif self.state == 4:
                    print("State 4: stop, 180 and grip")
                    self.sub.unregister()

                    # Rotation has been stopped, wait and then rotate 180 degrees
                    self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

                    print("Turning 180")
                    self.turn_amount(180)

                    # Drive backwards for 3 seconds
                    print("Drive backwards for 3 seconds")
                    self.drive_time(-0.05, 3)

                    # Grip the object
                    print("Grip object")
                    time.sleep(1)
                    self.gripper_angle = self.gripper_close
                    self.pub = rospy.Publisher("/servo", UInt16, queue_size=3)
                    self.pub.publish(self.gripper_angle)
                    time.sleep(self.gripper_time)

                    # Drive forward again for 3 seconds
                    print("Drive forward for 3 seconds")
                    self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
                    self.drive_time(0.05, 3)
                    self.state += 1

                # Follow line back to user
                elif self.state == 5:
                    print("State 5: Follow line back to user")
                    self.time_last = time.time()
                    self.sub.unregister()
                    self.sub = rospy.Subscriber("cmd_vel_mux/Line_Follower_vel", Twist, self.movement_callback)

                # Rotate 180, release object
                elif self.state == 6:
                    print("State 6: Rotate 180, release object")
                    self.sub.unregister()

                    print("Turning 180")
                    self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
                    self.turn_amount(180)

                    # release gripper
                    print("Release object")
                    self.gripper_angle = self.gripper_open
                    self.pub = rospy.Publisher("/servo", UInt16, queue_size=3)
                    self.pub.publish(self.gripper_angle)
                    time.sleep(self.gripper_time)
                    print("QUICK, TAKE YOUR DRINK NOW!")
                    time.sleep(self.takeaway_time)

                    print("Turning 180")
                    self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
                    self.turn_amount(180)
                    self.state = 1

                else:
                    print("undefined state")


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    node = DecisionNode()
    node.main()
