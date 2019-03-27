#!/usr/bin/env python2.7

import rospy
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from numpy import pi as PI
import time

"""
States
======
can be found at https:#www.draw.io/#G17kVh2GfapUA7j_Pc61OxIDDBFxVxKaeO tab 3
"""


def dummy_func():
    pass


def turnAmount(degrees):
    time.sleep(1)
    vel_com.angular.z = 30 * 2 * PI / 360
    pub.publish(vel_com)
    time.sleep(degrees / 30)
    vel_com.angular.z = 0
    pub.publish(vel_com)


def driveTime(speed, time):
    time.sleep(1)
    vel_com.linear.x = speed
    pub.publish(vel_com)
    time.sleep(time)
    vel_com.linear.x = 0
    pub.publish(vel_com)


def ArucoInstructionCallback(msg):
    detected_aruco = msg.data[0]
    if detected_aruco < aruco_threshold:
        aruco_to_find = detected_aruco
        print("Detected:", detected_aruco)
        state += 1


def movementCallback(msg):
    # when no line is detected
    if msg.linear.x == 0:
        line_counter_end = time.time()
    elif time.time() - time_last > 5:
        state += 1

    # publish to topic to turtlebot when a line is detected
    else:
        time_last = time.time()  # set clock
        pub.shutdown()
        pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        pub.publish(msg)

    # reset the counter, still moving


def ArucoDriveCallback(msg):
    # extract data from aruco node
    detected_aruco = msg.data[0]
    xpercentage = msg.data[1]
    if detected_aruco == aruco_to_find:
        print("Found:", detected_aruco)
        # check wether aruco is in the middle (with a margin))
        if 0.5 - MARGIN < xpercentage < 0.5 + MARGIN:
            # Stop turning and go to next state
            vel_com.angular.z = 0
            pub.shutdown()
            pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
            pub.publish(vel_com)
            state += 1
        # Drive up to the can


def main():
    print("Decision node started")
    # Margin for center of ArUco detection state 3
    MARGIN = 0.05
    vel_com = Twist()
    aruco_threshold = 4
    detected_aruco = Float32MultiArray()
    state = 1
    previous_state = 0
    time_last = 0
    elapsed_seconds = 0
    sub = rospy.Subscriber("dummy", Image, dummy_func)
    pub = rospy.Publisher("dummy", Image, queue_size=1)
    rospy.init_node("decision_node", anonymous=True)

    while True:
        # switch case for state machine
        # long time no subscribed -> end of line
        if state == 2:
            line_counter_end = time.time()  # line_counter_end = std::chrono::system_clock::now()
            elapsed_seconds = line_counter_end - line_counter_start  # elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(line_counter_end-line_counter_start).count()

            if elapsed_seconds >= 3:
                print("End of line reached, switch to aruco detection elapsed time:", elapsed_seconds)
                state += 1

        else:
            line_counter_start = time.time()  # line_counter_start = std::chrono::system_clock::now()

        if state != previous_state:
            # Scanning for ArUco instruction
            if state == 1:
                print("State 1: Scanning for ArUco", elapsed_seconds)
                sub.unregister()
                sub = rospy.Subscriber("/id_pub", Image,
                                       ArucoInstructionCallback)  # subscribing to ArUco ID topic and use callback
                previous_state = state

            # Move to end of line
            elif state == 2:
                print("State 2: Move to object", elapsed_seconds)
                sub.unregister()
                sub = rospy.Subscriber("cmd_vel_mux/Line_Follower_vel", Twist, movementCallback)
                previous_state = state
                # Turn around until something gets published on the node we're subscribed to
                # Meaning we turn 180 degrees until we see the yellow line, then go into the callback

            # Rotate while centering for correct ArUco code
            elif state == 3:
                print("State 3: Scanning for correct ArUco")
                sub.unregister()
                sub = rospy.Subscriber("/id_pub", Float32MultiArray, ArucoDriveCallback)
                # start turning (ArucoDriveCallback will stop the turning)
                vel_com.angular.z = 15 * 2 * PI / 360
                pub.shutdown()
                pub = rospy.Publisher("/movement_instruction", Twist, queue_size=3)
                pub.publish(vel_com)
                previous_state = state

                # Stop rotation, rotate 180 and drive backwards, then grip the object
            elif state == 4:
                print("State 4: stop, 180 and grip")
                sub.unregister()

                # Rotation has been stopped, wait and then rotate 180 degrees
                pub.shutdown()
                pub = rospy.Publisher("/movement_instruction", Twist, queue_size=3)

                turnAmount(180)

                # Drive backwards for 3 seconds
                driveTime(-1, 3)

                # Grip the object
                time.sleep(1)
                msg = "Griep!"
                pub.shutdown()
                pub = rospy.Publisher("gripper", String, queue_size=3)
                pub.publish(msg)
                # Drive forwards again for 3 seconds
                pub.shutdown()
                pub = rospy.Publisher("/movement_instruction", Twist, queue_size=3)
                driveTime(1, 3)
                state += 1
                previous_state = state
                # Follow line back to user

            elif state == 5:
                sub.unregister()
                sub = rospy.Subscriber("cmd_vel_mux/Line_Follower_vel", Twist, movementCallback)

            # Rotate 180, release object
            elif state == 6:
                sub.unregister()
                turnAmount(180)
                # release gripper
                turnAmount(180)
                state = 1

            else:
                print("undefined state")

        rospy.spin()
    return 0


main()
