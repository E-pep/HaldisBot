#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
PI = 3.1415926535897
FWD = 0
BWD = 1
LEFT = 2
RIGHT = 3
STOP = 4

### only used for capturing keystrokes, can be removed during integration ###
import sys, termios, tty, os, time
 
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
 
button_delay = 0.2
###  ###

def readKeyboard():
    # Starts a new node
	rospy.init_node('keyboard', anonymous=True)
	instruction_publisher = rospy.Publisher('/movement_instruction', Int32, queue_size=1)
    
	print("Move the robot with z and s, turn with q and d, quit with p:")

	while not rospy.is_shutdown():
		char = getch()
		if(char == "z"):
				# move forwards
			instruction_publisher.publish(FWD)
		elif(char == "s"):
				# move backwards
			instruction_publisher.publish(BWD)
		elif(char == "q"):
				# turn left
			instruction_publisher.publish(LEFT)
		elif(char == "d"):
				# turn right
			instruction_publisher.publish(RIGHT)
		elif(char == "e"):
				# stop robot
			instruction_publisher.publish(STOP)
		elif(char == "p"):
				# stop program
			instruction_publisher.publish(STOP)
			exit(0)

if __name__ == '__main__':
    try:
	readKeyboard()
    except rospy.ROSInterruptException: pass
