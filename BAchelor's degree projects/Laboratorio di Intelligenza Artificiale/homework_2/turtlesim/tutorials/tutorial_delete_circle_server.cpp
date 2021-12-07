#include <ros/ros.h>
#include <turtlesim/Kill.h>
#include <turtlesim/RemoveCircle.h>
#include <turtlesim/GetCircles.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>

std::vector<turtlesim::Circle> circles;

bool delete_turtle(turtlesim::RemoveCircle::Request &req, turtlesim::RemoveCircle::Response &res) {
	ROS_INFO("Request: id=%d", req.id);

	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<turtlesim::Kill>("kill");
	
	if (circles.size() == 0) {
		ros::ServiceClient client_get = nh.serviceClient<turtlesim::GetCircles>("get_circles");
		turtlesim::GetCircles srv_get;
	
		if (client_get.call(srv_get)) {
			ROS_INFO("Chiamo il servizio get_circles");
		}
		else {
			ROS_ERROR("Impossibile chiamare il servizio tutorial_get_circle");
			return 1;
		}
		
		circles = srv_get.response.circles;
	}

	turtlesim::Kill srv;
	std::string name = "turtle";
	char string_id[8];
	sprintf(string_id, "%d", req.id);
	srv.request.name = name + string_id;
	

	if(client.call(srv)) {
		ROS_INFO("Sto cancellando la tartaruga %s", srv.request.name.c_str());
	}
	else {
		ROS_ERROR("Impossibile chiamare kill");
		return 1;
	}

	int i;
	for(i=0; i<circles.size(); i++) {
		if(circles[i].id == req.id) {
			circles.erase(circles.begin()+i);
			continue;
		}
	}
	res.circles =  circles;
	ROS_INFO("Response: ");
	
	for (i=0; i<res.circles.size(); i++) {
		ROS_INFO(">>>id:%d, x=%f, y=%f", res.circles[i].id, res.circles[i].x, res.circles[i].y);
	}
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "tutorial_delete_circle_server");

	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("delete_circle", delete_turtle);
	ROS_INFO("Ready to delete the turtle");

	ros::spin();

	return 0;
}
