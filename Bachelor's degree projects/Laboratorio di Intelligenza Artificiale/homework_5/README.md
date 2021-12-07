The goal of this homework was to use ActionLib and implement the following behaviour: the move_base_client should save the initial position of the robot, wait the action server to be ready to satisfy his requests, then it should send a goal to the action server, wait 30 seconds and then cancel its request. Afterwards, the action client should send another goal with the initial coordinates and wait for the action server to send him the result of his request. So the robot should start moving towards a goal, then stop and go back to its initial position.

To execute the move_base_client you need to:
* Run roscore
* Run stage_ros executing the command:
	```
	rosrun stage_ros stageros worlds/willow-erratic.world
	```
* Have a map of the environment to share with all ROS nodes through
	```
	rosrun map_server map_server mappa.yaml
	```
* Start the localizer and the planner through
	```
	rosrun thin_navigation thin_localizer_node
	rosrun thin_navigation thin_planner_node
	```
* Start rviz and add the map, poseArray and path to the configuration
* Write the command
	```
	rostopic pub /initialpose tab tab
	```
  and complete the obtained structure using
  	position: x=0.01, y=0.01
	orientation: w=6.12323399574e-17 (value obtained running *rostopic echo /base_pose_ground_truth*)
* Start the node with
	```
	rosrun actionlib_tutorials move_base_client
	```
