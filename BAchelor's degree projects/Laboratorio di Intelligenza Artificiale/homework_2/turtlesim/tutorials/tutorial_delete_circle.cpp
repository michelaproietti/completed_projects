#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Circle.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/RemoveCircle.h>
#include <turtlesim/GetCircles.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

turtlesim::Pose turtle_pose;
std::vector<turtlesim::Circle> circles;

void poseCallback(const turtlesim::Pose& pose)
{
  turtle_pose = pose;
}

void turtle_control() {
	int i;
	for(i=0; i<circles.size(); i++) {
		if (circles[i].x < turtle_pose.x+0.5 && circles[i].x > turtle_pose.x-0.5 && 
				circles[i].y < turtle_pose.y+0.5 && circles[i].y > turtle_pose.y-0.5) {
			ros::NodeHandle nh;
			ros::ServiceClient client = nh.serviceClient<turtlesim::RemoveCircle>("delete_circle");
			turtlesim::RemoveCircle srv;
			srv.request.id = circles[i].id;

			ROS_INFO("Richiesta delete della tartaruga %d", srv.request.id);

			if(client.call(srv)) {
				ROS_INFO("Chiamo servizio delete_circle");
			}
			else {
				ROS_ERROR("Impossibile chiamare il servizio tutorial_delete_circle");
				return;
			}
			ROS_INFO("Cancellata tartaruga %d", srv.request.id);
			circles.erase(circles.begin()+i);
			
			ROS_INFO(">>circles:");
			for (i=0; i<circles.size(); i++) {
				ROS_INFO(">>>id:%d, x=%f, y=%f", circles[i].id, circles[i].x, circles[i].y);
			}

			return;
		}
	}
	return;
}

int main (int argc, char** argv) {
	ros::init(argc, argv, "tutorial_delete_circle_client");

	ros::NodeHandle nh;

	ros::ServiceClient client_get = nh.serviceClient<turtlesim::GetCircles>("get_circles");
	ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1, poseCallback);

	turtlesim::GetCircles srv_get;
		
	if (client_get.call(srv_get)) {
		ROS_INFO("Chiamo il servizio get_circles");
	}
	else {
		ROS_ERROR("Impossibile chiamare il servizio tutorial_get_circle");
		return 1;
	}
	circles = srv_get.response.circles;

	while(ros::ok()) {
		if (circles.size() == 0) {
			ROS_INFO("Non ci sono tartarughe da cancellare");
			return 0;
		}

		turtle_control();
		
		ros::spinOnce();
	}

	return 0;
}
