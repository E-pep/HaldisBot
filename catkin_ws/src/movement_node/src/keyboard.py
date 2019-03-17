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
	instruction = 4

	while not rospy.is_shutdown():
		char = getch()
		if(char == "z"):
				# move forwards
			instruction = FWD
		elif(char == "s"):
				# move backwards
			instruction = BWD
		elif(char == "q"):
				# turn left
			instruction = LEFT
		elif(char == "d"):
				# turn right
			instruction = RIGHT
		elif(char == "e"):
				# stop robot
			instruction = STOP
		elif(char == "p"):
				# stop program
			instruction = STOP
			instruction_publisher.publish(instruction)
			exit(0)
		print(instruction)
		instruction_publisher.publish(instruction)

if __name__ == '__main__':
    try:
	readKeyboard()
    except rospy.ROSInterruptException: pass
