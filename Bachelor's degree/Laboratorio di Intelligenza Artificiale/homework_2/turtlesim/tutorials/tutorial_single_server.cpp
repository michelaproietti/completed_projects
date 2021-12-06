#include <ros/ros.h>
#include <turtlesim/Circle.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/SpawnCircle.h>
#include <turtlesim/GetCircles.h>
#include <turtlesim/Kill.h>
#include <turtlesim/RemoveCircle.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>

//variabile globale per fare in modo che le tartarughe abbiano tutte id diversi
int id = 2;

std::vector<turtlesim::Circle> circles;

bool spawn_callback(turtlesim::SpawnCircle::Request &req, turtlesim::SpawnCircle::Response &res) {
	ROS_INFO("<<<SPAWN_CIRCLE>>>");
	ROS_INFO("Request: x=%f, y=%f", (float)req.x, (float)req.y);

	ros::NodeHandle nh;
	ros::service::waitForService("spawn");
	ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("spawn");

	srand((unsigned)time(NULL));
	int N = rand()%10+1;

	int i;
	for (i=0; i<N; i++) {
		
		turtlesim::Spawn srv;
		srv.request.x = req.x+rand()%11;
		srv.request.y = req.y+rand()%11;
		srv.request.theta = rand();

		if(client.call(srv)) {
			ROS_INFO("Sto disegnando la tartaruga %s", srv.response.name.c_str());
		}
		else {
			ROS_ERROR("Impossibile chiamare spawn");
			return 1;
		}	

		turtlesim::Circle c;
		c.id = id++;
		c.x = srv.request.x;
		c.y = srv.request.y;

		circles.push_back(c);
	}
	res.circles = circles;

	ROS_INFO ("Response:");
	for (i=0; i<res.circles.size(); i++) {
		ROS_INFO(">>>id:%d, x=%f, y=%f", res.circles[i].id, res.circles[i].x, res.circles[i].y);
	}


	return true;
}


bool get_callback(turtlesim::GetCircles::Request &req, turtlesim::GetCircles::Response &res) {
	ROS_INFO("<<<GET_CIRCLES>>>");

	res.circles = circles;
	int i;
	ROS_INFO("Response:");
	
	for (i=0; i<res.circles.size(); i++) {
		ROS_INFO(">>>id:%d, x=%f, y=%f", res.circles[i].id, res.circles[i].x, res.circles[i].y);
	}

	return true;
}

bool delete_callback(turtlesim::RemoveCircle::Request &req, turtlesim::RemoveCircle::Response &res) {
	ROS_INFO("<<<DELETE_CIRCLE>>>");

	ROS_INFO("Request: id=%d", req.id);

	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<turtlesim::Kill>("kill");

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
	if (circles.empty()) {
		ROS_INFO("empty");
	}
	else {
		for (i=0; i<res.circles.size(); i++) {
			ROS_INFO(">>>id:%d, x=%f, y=%f", res.circles[i].id, res.circles[i].x, res.circles[i].y);
		}
	}
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "tutorial_draw_circle_server");
	ros::NodeHandle nh;
	
	ros::ServiceServer service_spawn = nh.advertiseService("spawn_circle", spawn_callback);

	ros::ServiceServer service_get = nh.advertiseService("get_circles", get_callback);

	ros::ServiceServer service_delete = nh.advertiseService("delete_circle", delete_callback);
	ROS_INFO("Single server ready");

	ros::spin();

	return 0;
}
