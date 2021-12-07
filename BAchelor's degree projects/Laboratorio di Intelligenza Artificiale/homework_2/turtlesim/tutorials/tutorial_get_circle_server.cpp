#include <ros/ros.h>
#include <turtlesim/SpawnCircle.h>
#include <turtlesim/GetCircles.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>


bool get_circle(turtlesim::GetCircles::Request &req, turtlesim::GetCircles::Response &res) {
	ros::NodeHandle nh;
	ros::service::waitForService("spawn_circle");
	ros::ServiceClient client = nh.serviceClient<turtlesim::SpawnCircle>("spawn_circle");

	//faccio una finta chiamata al servizio SpawnCircle per ricevere circles in risposta
	turtlesim::SpawnCircle srv;
	srv.request.x = 0;
	srv.request.y = 0;

	if(client.call(srv)) {
		ROS_INFO("Chiamo servizio tutorial_draw_circle_server");
	}
	else {
		ROS_ERROR("Impossibile chiamare il servizio tutorial_draw_circle_server");
		return 1;
	}

	res.circles = srv.response.circles;
	int i;
	ROS_INFO("Response: ");
	
	for (i=0; i<res.circles.size(); i++) {
		ROS_INFO(">>>id:%d, x=%f, y=%f", res.circles[i].id, res.circles[i].x, res.circles[i].y);
	}


	return true;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "tutorial_get_circle_server");
	ros::NodeHandle nh;
	
	ros::ServiceServer service = nh.advertiseService("get_circles", get_circle);
	ROS_INFO("Ready to get turtle");

	ros::spin();

	return 0;
}
