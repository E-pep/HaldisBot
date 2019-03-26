#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <sstream>

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

                /// Move to object
                case 2 :
                    std::cout << "State 2: Move to object" << std::endl;
                    sub.shutdown();
                    previous_state = state;
                    sub = n.subscribe("cmd_vel_mux/Line_Follower_vel", 1, movementCallback);
                    // Turn around until something gets published on the node we're subscribed to
                    // Meaning we turn 180 degrees until we see the yellow line, then go into the callback
                    break;

                /// Scanning for correct ArUco code
                case 3 :
                    std::cout << "State 3: Scanning for correct ArUco" << std::endl; //
                    sub.shutdown();
                    previous_state = state;
                    sub = n.subscribe("/id_pub", 1, ArucoCallback2);
                    break;

                /// Grip object
                case 4 :
                    std::cout << "State 4: Grip the object" << std::endl; //
                    sub.shutdown();
                    previous_state = state;
                    //sub = n.subscribe("", 1, GripperCallback);
                    break;
                case 5 :
                    sub = n.subscribe("cmd_vel_mux/Line_Follower_vel", 1, movementCallback);
                    while(nodet < NODET){
                        if(!foundline){
                            nodet++;
                        }
                    }
                    state = 6;
                case 6 :

                    vel_com.angular = [0.0,0.0,pi/2];
                    vel_com.linear = [0.0, 0.0, 0.0];
                    vel_command = vel_com->data;
                    wait(2)
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
