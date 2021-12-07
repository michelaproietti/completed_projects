#include <ros/ros.h>
#include <turtlesim/Circle.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/SpawnCircle.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

//variabile globale per fare in modo che le tartarughe abbiano tutte id diversi
int id = 2;

std::vector<turtlesim::Circle> circles;

bool spawn_turtle(turtlesim::SpawnCircle::Request &req, turtlesim::SpawnCircle::Response &res) {
	if (req.x == 0 && req.y==0) {
		res.circles = circles;
		return true;
	}

	ROS_INFO("Request: x=%f, y=%f", (float)req.x, (float)req.y);

	ros::NodeHandle nh;
	ros::service::waitForService("spawn");
	ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("spawn");

	srand((unsigned)time(NULL));
	
	//imposto randomicamente il numero N di tartarughe da spawnare
	int N = rand()%10+1;
	int i;


	for (i=0; i<N; i++) {
		//imposto i parametri ed effettua la chiamata al servizio Spawn per visualizzare le tartarughe
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


int main(int argc, char** argv) {
	ros::init(argc, argv, "tutorial_draw_circle_server");
	ros::NodeHandle nh;
	
	ros::ServiceServer service = nh.advertiseService("spawn_circle", spawn_turtle);
	ROS_INFO("Ready to draw the circle");

	ros::spin();

	return 0;
}
