#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/SpawnCircle.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>


int main (int argc, char** argv) {
	ros::init(argc, argv, "tutorial_draw_circle_client");

	ros::NodeHandle nh;
	ros::service::waitForService("spawn_circle");
	ros::ServiceClient client = nh.serviceClient<turtlesim::SpawnCircle>("spawn_circle");

	//imposto i margini entro cui spawnare le tartarughe in modo che non vengano tagliate
	float x = 0.5;
	float y = 0.5;
	ROS_INFO("Richiesta spawn_circle entro i margini: x=%f, y=%f", x, y);

	//imposto i parametri e chiamo il servizio SpawnCircle
	turtlesim::SpawnCircle srv;
	srv.request.x = x;
	srv.request.y = y;

	if(client.call(srv)) {
		ROS_INFO("Chiamo servizio spawn_circle");
	}
	else {
		ROS_ERROR("Impossibile chiamare il servizio tutorial_draw_circle");
		return 1;
	}


	return 0;
}
