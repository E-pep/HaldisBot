#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <sstream>
#include <unistd.h>

/* states

can be found at https://www.draw.io/#G17kVh2GfapUA7j_Pc61OxIDDBFxVxKaeO tab 3

*/

#define NODET 100
geometry_msgs::Twist::ConstPtr& vel_com;
bool foundline = false;
int nodet = 0;
int aruco_threshold = 4;
int detected_aruco;
int aruco_to_find;
int state = 1;
int previous_state = 0;

ros::Subscriber sub;
ros::Publisher pub;


void ArucoCallback(const std_msgs::Int32::ConstPtr& msg)
{
	detected_aruco = msg->data;

	if(detected_aruco < aruco_threshold)
	{
		std::cout << "Detected: "<< detected_aruco << std::endl;
		aruco_to_find = detected_aruco;
		state++;
    }
}

void movementCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    Vector3 vel_command = msg->data;

    if(msg->data.linear == 0){
         foundline = false;
    }
    else{
        foundline = true;
        vel_command = msg->data;
    }

    // We receive the velocity command from Jarrit's line follower
    // Then we publish it to the movement node, which allows us to follow the line

    // TODO: How do we detect the end of the line? Put ArUco marker down? Put down a piece of tape in another colour and make Jarrit publish a certain value?
    /*
    if(endofline)
    {
        state++;
    }
    */
}

void ArucoCallback2(const std_msgs::Int32::ConstPtr& msg)
{
	detected_aruco = msg->data;

	if(detected_aruco == aruco_to_find)
	{
		std::cout << "Found: "<< detected_aruco << std::endl;
		state++;
		// Drive up to the can
    }
}



int main(int argc, char **argv)
{
	std::cout << "Decision node started" << std::endl;

	ros::init(argc, argv, "listener");
	ros::NodeHandle n;

	while(true)
	{
		// switch case for state machine
        if(state != previous_state)
        {
            switch(state)
            {
                /// Scanning for ArUco instruction
                case 1 :
                    std::cout << "State 1: Scanning for ArUco" << std::endl;
                    sub = n.subscribe("/id_pub", 1, ArucoCallback);	        // subscribing to ArUco ID topic and use callback
                    previous_state = state;
                    break;

                /// Move to end of line
                case 2 :
                    std::cout << "State 2: Move to object" << std::endl;
                    sub.shutdown();
                    previous_state = state;
                    sub = n.subscribe("cmd_vel_mux/Line_Follower_vel", 1, movementCallback);
                    // Turn around until something gets published on the node we're subscribed to
                    // Meaning we turn 180 degrees until we see the yellow line, then go into the callback
                    break;

                /// Rotate while centering for correct ArUco code
                case 3 :
                    std::cout << "State 3: Scanning for correct ArUco" << std::endl; //
                    sub.shutdown();
                    previous_state = state;
                    sub = n.subscribe("/id_pub", 1, ArucoCallback2);
                    break;

                /// Stop rotation, rotate 180 and drive backwards, then grip the object
                case 4 :
                    std::cout << "State 4: stop, 180 and grip" << std::endl; //
                    sub.shutdown();
                    previous_state = state;

                    // Rotation has been stopped, wait and then rotate 180°
                    pub = n.advertise<geometry_msgs::Twist>("/movement_instruction", 3);

                    sleep(1);
                    vel_com.angular.z = 30 * 2*pi/360;
                    vel_command = vel_com->data;
                    pub.publish(vel_command);
                    ros::spinOnce();
                    sleep(6);

                    vel_com.angular.z = 0;
                    vel_command = vel_com->data;
                    pub.publish(vel_command);
                    ros::spinOnce();

                    // Drive backwards for 3 seconds
                    sleep(1);
                    vel_com.linear.x = -1;
                    vel_command = vel_com->data;
                    pub.publish(vel_command);
                    ros::spinOnce();
                    sleep(3);

                    vel_com.linear.x = 0;
                    vel_command = vel_com->data;
                    pub.publish(vel_command);
                    ros::spinOnce();

                    // Grip the object
                    sleep(1);
                    pub = n.advertise<std_msgs::String>("gripper", 3);
                    std_msgs::String msg.data = "Grip!";
                    pub.publish(msg);
                    ros::spinOnce();

                    // Drive forwards again for 3 seconds
                    pub = n.advertise<geometry_msgs::Twist>("/movement_instruction", 3);

                    sleep(1);
                    vel_com.linear.x = 1;
                    vel_command = vel_com->data;
                    pub.publish(vel_command);
                    ros::spinOnce();
                    sleep(3);

                    vel_com.linear.x = 0;
                    vel_command = vel_com->data;
                    pub.publish(vel_command);
                    ros::spinOnce();

                    state++;
                    break;

                /// Follow line back to user
                case 5 :
                    sub.shutdown();
                    sub = n.subscribe("cmd_vel_mux/Line_Follower_vel", 1, movementCallback);
                    while(nodet < NODET){
                        if(!foundline){
                            nodet++;
                        }
                    }
                    state++;
                    break;

                /// Rotate 180, release object
                case 6 :
                    vel_com.angular = [0.0,0.0,pi/2];
                    vel_com.linear = [0.0, 0.0, 0.0];
                    vel_command = vel_com->data;
                    sleep(2)
                    vel_com.angular = [0.0,0.0,0.0];
                    vel_com.linear = [0.0, 0.0, 0.0];
                    vel_command = vel_com->data;
                    state = 1;

                default:
                    std::cout << "undefined state" << std::endl;
                    break;
            }
        }
        ros::spinOnce();
    }
	return 0;
}
