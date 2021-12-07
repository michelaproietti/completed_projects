#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
	ros::init(argc, argv, "move_base_client");
	
	//save the initial pose
	ros::NodeHandle nh;
	boost::shared_ptr<nav_msgs::Odometry const> msg_ptr;
	nav_msgs::Odometry initial_pose;
	msg_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", nh);
	if (msg_ptr != NULL) {
		initial_pose = *msg_ptr;
	}


	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("waiting for the move_base action server to come up");
	}


	//send a target goal to move_base
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose .header.frame_id = "move_base" ;
	goal.target_pose .header.stamp = ros::Time::now();

	goal.target_pose .pose.position .x = 7.0;
	goal.target_pose .pose.position .y = 1.5;
	goal.target_pose .pose.orientation .z = 1.0;

	ROS_INFO ("Sending goal");
	ac.sendGoal (goal);

	//wait for 10 seconds
	ROS_INFO ("Waiting for 30 seconds");
	ros::Duration(30).sleep();

	//delete the target goal
	ROS_INFO ("Deleting goal");
	ac.cancelAllGoals();

	//return to initial_pose
	ROS_INFO ("Sending goal to go back to the initial position");
	move_base_msgs::MoveBaseGoal goal_origin;

	goal_origin.target_pose .header.frame_id = "move_base" ;
	goal_origin.target_pose .header.stamp = ros::Time::now();

	goal_origin.target_pose .pose.position .x = initial_pose.pose.pose.position.x;
	goal_origin.target_pose .pose.position .y = initial_pose.pose.pose.position.y;
	goal_origin.target_pose .pose.position .z = initial_pose.pose.pose.position.z;
	goal_origin.target_pose .pose.orientation .x = initial_pose.pose.pose.orientation.x;
	goal_origin.target_pose .pose.orientation .y = initial_pose.pose.pose.orientation.y;
	goal_origin.target_pose .pose.orientation .z = initial_pose.pose.pose.orientation.z;
	goal_origin.target_pose .pose.orientation .w = initial_pose.pose.pose.orientation.w;

	ROS_INFO ("Sending goal");
	ac.sendGoal (goal_origin);

	ac.waitForResult();
	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Got back to the initial position");
	else 
		ROS_INFO("The base tried to go back to its initial position and got close to it");

	return 0;
}
