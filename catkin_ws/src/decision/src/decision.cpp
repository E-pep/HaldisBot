#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <sstream>

/* states

0: waiting on the Aruco order
1: Move to objects


*/


int aruco_threshold = 4;
int detected_aruco;
int state = 0;
int previous_state = state;


void ArucoCallback(const std_msgs::Int32::ConstPtr& msg)
{
	detected_aruco = msg->data;

	if(detected_aruco < aruco_threshold)
	{
		std::cout << "detected:"<< detected_aruco << std::endl;
		previous_state = state;
		state = 1;	
	}


	
}



int main(int argc, char **argv)
{
	std::cout << "decision node started" << std::endl;

	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub;
	sub = n.subscribe("/id_pub", 1, ArucoCallback);

	while(1)
	{
		// switch case for state machine 
	 if(state != previous_state)
	 {
		switch(state) 
		{
		case 0 : std::cout << "state 0" << std::endl; 			// waiting for Aruco
			 sub = n.subscribe("/id_pub", 1, ArucoCallback);	// subscribing to Aruco topic and use callback
			 break;

    		case 1 : std::cout << "state 1" << std::endl; 			// Move to Object 
			 sub.shutdown();
			 previous_state = state;
			// sub = nh.subscribe("", 1, callbackFunc);
			 break;        
    		case 2 : std::cout << "state 2" << std::endl; // 
             		 break;
    		case 3 : std::cout << "state 3" << std::endl; // 
			 break;        

		default: std::cout << "undefined state" << std::endl;
			 break;
		}

	 }
	 ros::spinOnce();

	}


	return 0;
}
