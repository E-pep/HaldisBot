#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <sstream>
#include <chrono>
#include <unistd.h>
#include <math.h>

/* states

can be found at https://www.draw.io/#G17kVh2GfapUA7j_Pc61OxIDDBFxVxKaeO tab 3

*/

//Margin for center of aruca detection state 3
const float MARGIN = 0.05;
//timer until no line found state 2

geometry_msgs::Twist vel_com;
int aruco_threshold = 4;
std_msgs::Float32MultiArray detected_aruco;
int aruco_to_find;
int state = 1;
int previous_state = 0;

std::chrono::time_point<std::chrono::system_clock> line_counter_start, line_counter_end;

ros::Subscriber sub;
ros::Publisher pub;

void turnAmount(int degrees)
{
    sleep(1);
    vel_com.angular.z = 30 * 2*M_PI/360;
    pub.publish(vel_com);
    sleep(degrees/30);
    vel_com.angular.z = 0;
    pub.publish(vel_com);
}

void driveTime(int speed, int time)
{
    sleep(1);
    vel_com.linear.x = speed;
    pub.publish(vel_com);
    sleep(time);
    vel_com.linear.x = 0;
    pub.publish(vel_com);
}


void ArucoInstructionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
  	detected_aruco = msg.data;
  	if(detected_aruco < aruco_threshold)
    {
		std::cout << "Detected: "<< detected_aruco << std::endl;
		aruco_to_find = detected_aruco;
		state++;
    }
}

void movementCallback(const geometry_msgs::Twist msg)
{
    //when no line is detected
    if(msg.linear.x == 0)
    {
        line_counter_end = system_clock.now();
        if(line_counter_end - line_counter_start > 5)
        {
            state++;
        }
    }
    //publish to topic to turtlebot when a line is detected
    else
    {
        line_counter_start = system_clock.now(); //set clock
        ros::NodeHandle publish_handle;
        pub = publish_handle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        pub.publish(msg);
    }
    // reset the counter, still moving
}

void ArucoDriveCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // extract data from aruco node
    detected_aruco = msg->data[0];
    float xpercentage = msg->data[1];
  	if(detected_aruco == aruco_to_find)
  	{
    	std::cout << "Found: "<< detected_aruco << std::endl;
        // check wether aruco is in the middle (with a margin))
        if(xpercentage < 0.5+MARGIN && xpercentage > 0.5-MARGIN)
        {
            // Stop turning and go to next state
            vel_com.angular.z = 0;

            pub.publish(vel_com);
            state++;
        }
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
                    sub = n.subscribe("/id_pub", 1, ArucoInstructionCallback);	        // subscribing to ArUco ID topic and use callback
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
                    sub = n.subscribe("/id_pub", 1, ArucoDriveCallback);
                    // start turning (ArucoDriveCallback will stop the turning)
                    vel_com.angular.z = 15 * 2*M_PI/360;

                    pub.publish(vel_com);
                    break;

                /// Stop rotation, rotate 180 and drive backwards, then grip the object
                case 4 :
                    std::cout << "State 4: stop, 180 and grip" << std::endl; //
                    sub.shutdown();
                    previous_state = state;

                    // Rotation has been stopped, wait and then rotate 180°
                    pub.shutdown();
                    pub = n.advertise<geometry_msgs::Twist>("/movement_instruction", 3);

                    sleep(1);
                    turnAmount(180);

                    // Drive backwards for 3 seconds
                    driveTime(-1, 3);

                    // Grip the object
                    pub.shutdown();
                    pub = n.advertise<std_msgs::String>("gripper", 3);

                    sleep(1);
                    std_msgs::String msg.data = "Grip!";
                    pub.publish(msg);
                    pub.shutdown();

                    // Drive forwards again for 3 seconds
                    pub.shutdown();
                    pub = n.advertise<geometry_msgs::Twist>("/movement_instruction", 3);

                    driveTime(1, 3);

                    state++;
                    break;

                /// Follow line back to user
                case 5 :
                    sub.shutdown();
                    sub = n.subscribe("cmd_vel_mux/Line_Follower_vel", 1, movementCallback);
                    break;

                /// Rotate 180, release object
                case 6 :
                    sub.shutdown();
                    turnAmount(180);
                    //release gripper
                    turnAmount(180);
                    state = 1;
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
/*

*/
