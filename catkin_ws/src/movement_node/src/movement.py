#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

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

def move():
    # Starts a new node
	rospy.init_node('move', anonymous=True)
	velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()
    
    #Receiveing the user's input
	print("Move the robot with z and s, turn with q and d, quit with p:")
	speed = 1						#1 unit per sec
	angular_speed = 30*2*PI/360		#30 degrees per sec

    #Initialize values
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
    
	while not rospy.is_shutdown():
		char = getch()
		if(char == "z"):
				#move forwards
			vel_msg.linear.x = abs(speed)
			velocity_publisher.publish(vel_msg)
			time.sleep(button_delay)
		if(char == "s"):
				#move backwards
			vel_msg.linear.x = -abs(speed)
			velocity_publisher.publish(vel_msg)
			time.sleep(button_delay)
		if(char == "q"):
				#turn left
			vel_msg.angular.z = abs(angular_speed)
			velocity_publisher.publish(vel_msg)
			time.sleep(button_delay)
		if(char == "d"):
				#turn right
			vel_msg.angular.z = -abs(angular_speed)
			velocity_publisher.publish(vel_msg)
			time.sleep(button_delay)
		if(char == "p"):
				#stop
			exit(0)
		else:
				#stop robot
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
			velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
	move()
    except rospy.ROSInterruptException: pass
