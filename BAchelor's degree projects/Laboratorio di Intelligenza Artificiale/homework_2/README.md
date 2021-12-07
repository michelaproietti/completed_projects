In this homework, I used Turtlesim, which is the first ROS robot simulation.
Initially, the Turtlesim environment contains just one single turtle.
The aim of this project was to first spawn N circles, with N given as input by the user, and 
then make the initial turtle move in the environment and delete any other turtle hit by it.

Two versions have been uploaded.
The first one has 3 separate servers:
* tutorial_draw_circle_server.cpp
* tutorial_delete_circle_server.cpp
* tutorial_get_circle_server.cpp
and two clients:
* tutorial_draw_circle.cpp
* tutorial_delete_circle.cpp
This version does not contain a client for getCircles, since this service is called in
tutorial_delete_circle. Using 3 separate servers there was an issue, related to the need of
communicating the changes made to the array of circles from one ROS node to another.

To solve this problem, a second version has been developed, in which there is a single server
that provides all services and three different clients. This version allows us to have a global
array of circles so that we always know what are the turtles that are still alive both from
draw_circle, that adds the new turtles as soon as they are spawned, and delete_circle, that deletes
the turtles from the array when they are hit.

To test the homework, you need to compile using catkin build and then:
* Open a terminal where we execute the command
	```
	$ roscore
	```
* Open a terminal where we execute the command
	```
	$ rosrun turtlesim turtlesim_node
	```
* Open a terminal where we start the server with the command
	```
	$ rosrun turtlesim tutorial_single_server
	```
  that will allow us to move with the turtle to test the delete function.
* Open a terminal used to execute the clients through the following commands
	```
	$ rosrun turtlesim tutorial_draw_circle
	$ rosrun turtlesim tutorial_get_circle
	$ rosrun turtlesim tutorial_delete_circle
	```
