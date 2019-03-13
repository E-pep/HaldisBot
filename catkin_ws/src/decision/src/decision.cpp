#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <sstream>


int detected_aruco;


void ArucoCallback(const std_msgs::Int32::ConstPtr& msg)
{
	detected_aruco = msg->data;
	std::cout << "detected:"<< detected_aruco << std::endl;
}

int main(int argc, char **argv)
{
	std::cout << "decision node started" << std::endl;

	ros::init(argc, argv, "listener");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/id_pub", 1000, ArucoCallback);

	ros::spin();


	return 0;
}
