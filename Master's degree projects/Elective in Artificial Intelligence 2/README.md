# Elective in Artificial Intelligence

## [Click here to watch a video summary](https://www.youtube.com/watch?v=XQVYMuwKDL8)

**Note**: Due to HTML versions mismatches (the robot software is quite old) the HTML pages did not change on the robot's tablet, so we played the game on an external computer while still implementing the verbal and gestural interaction on the robot.

## Introduction

The purpose of the project is to develop a social and interactive robot for people’s entertainment. It allows to play a cooperative version of the Tower of Hanoi game, in which the user and the robot execute a move alternately. The robot is also able to advise the person on the most appropriate level for him, by collecting some data about the user through an initial phase of questions. Communication between the two agents has been implemented in different ways. Several frameworks were exploited, including pepper_tools, AIPlan4EU, three.js and additional web development tools in order to carry out the robot’s relational model, its intelligence to solve the game and a visual communication interface for the user in order to interact with the game.

![Pepper robot](https://th.bing.com/th/id/R.e60021353905ca6760f03bbc100c2f8c?rik=j%2f2wtT0pY0WCyQ&riu=http%3a%2f%2finfozene.com%2fblogging%2fwp-content%2fuploads%2f2017%2f09%2fSoftbank-humanoid-pepper-robot-427x640.jpg&ehk=rwkBpazEpC3zfepxlsev%2b%2fwZp3ZIWmRc59exsirb9EU%3d&risl=&pid=ImgRaw&r=0)

## Tools and Languages
This project has been mainly developed through Python and web development tools. Web development tools, including frameworks such as three.js and jQuery, allowed the creation of a graphical environment in which the user can visualize the progress of the game and interact to make the next move for solving the problem. Additionally, NAOqi has been used as it provides APIs to directly connect the Pepper robot to the server that sends it the directives to execute, such as when and what to say, how to move and so on. The functioning of these features has been tested through a virtual Pepper robot. This is possible through the use of the Pepper SDK plugin for Android Studio. It provides a set of graphical tools and a Java library, the QiSDK, in order to virtualize and visualize a Pepper robot and almost all of its functionalities. Furthermore, a Python framework called AIPlan4EU has been exploited to execute the planning in order to make the robot able to correctly choose the best next move to make. Finally, universal websockets have been used to ensure communication between the planner in Python, the robot software, and the website managed by JavaScript.

## Authors:
* Luca Polenta
* Michela Proietti
* Sofia Santilli
