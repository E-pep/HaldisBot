#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <sstream>
#include <chrono>

/* states

0: waiting on the Aruco order
1: Move to objects


*/


int aruco_threshold = 4;
int detected_aruco;
int aruco_to_find;
int state = 1;
int previous_state = 0;

std::chrono::time_point<std::chrono::system_clock> line_counter_start , line_counter_end;

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
    

    //publish to topic to turtlebot 
    ros::NodeHandle publish_handle;
    pub = publish_handle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pub.publish(msg);

    // reset the counter, still moving
    line_counter_start = std::chrono::system_clock::now();
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

	int elapsed_seconds;

	while(true)
	{
		// switch case for state machine


	//long time no subscribed -> end of line


	if(state == 2)
	{

		line_counter_end = std::chrono::system_clock::now();
		elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(line_counter_end-line_counter_start).count();

		if(elapsed_seconds >= 3)
		{
                    std::cout << "End of line reached, switch to aruco detection elapsed time: "<< elapsed_seconds << std::endl;	
		    state++;				
		}
	}
	else
	{
		line_counter_start = std::chrono::system_clock::now();	
	}


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

                default:
                    std::cout << "undefined state" << std::endl;
                    break;
            }
        }
        ros::spinOnce();
    }
	return 0;
}
