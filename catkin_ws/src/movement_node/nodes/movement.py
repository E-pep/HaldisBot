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
vel_msg = Twist()

def move(instruction):
	speed = 0.07					#1 unit per sec
	angular_speed = 30*2*PI/360		#30 degrees per sec
	vel_msg = Twist()
	if(instruction.data == FWD):
			#move forwards
		print("Driving forwards...")
		vel_msg.linear.x = abs(speed)
		vel_msg.angular.z = 0
		print("Driving forwards...")
	elif(instruction.data == BWD):
			#move backwards
		vel_msg.linear.x = -abs(speed)
		vel_msg.angular.z = 0
		print("Driving backwards...")
	elif(instruction.data == LEFT):
			#turn left
		vel_msg.linear.x = 0
		vel_msg.angular.z = abs(angular_speed)
		print("Turning left...")
	elif(instruction.data == RIGHT):
			#turn right
		vel_msg.linear.x = 0
		vel_msg.angular.z = -abs(angular_speed)
		print("Turning right...")
	else:
			#stop robot
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		print("Stopping robot...")
	velocity_publisher.publish(vel_msg)
	print("published")

    # Starts a new node
rospy.init_node('movement', anonymous=True)
instruction_subscriber = rospy.Subscriber('/movement_instruction', Int32, move)
velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
print("Move the robot with z and s, turn with q and d, quit with p:")
rospy.spin()


