#include <ros/ros.h>
#include <turtlesim/GetCircles.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>


int main (int argc, char** argv) {
	ros::init(argc, argv, "tutorial_get_circle_client");

	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<turtlesim::GetCircles>("get_circles");
	turtlesim::GetCircles srv;

	if(client.call(srv)) {
		ROS_INFO("Chiamo servizio get_circles");
	}
	else {
		ROS_ERROR("Impossibile chiamare il servizio tutorial_get_circle");
		return 1;
	}


	return 0;
}
