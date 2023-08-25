# Completed projects

This repository contains all the projects I took part in, grouped according to the context in which they were developed.

## Table of contents
* [Bachelor's degree projects](https://github.com/michelaproietti/completed_projects/tree/main/Bachelor's%20degree%20projects)
* [Master's degree projects](https://github.com/michelaproietti/completed_projects/tree/main/Master's%20degree%20projects)

## Bachelor's degree projects
This folder contains the projects realized during the Bachelor's degree in Computer and Automation Engineering at Sapienza University of Rome. The following list contains the names of the courses and a brief description of the projects:

* **Laboratorio di Intelligenza Artificiale (AI Laboratory)**: set of 5 homeworks aimed at solving different tasks, namely using Orazio firmware for robotic mobile bases, executing different tasks using Turtlesim, and implementing an image classification algorithm. These homeworks were developed using different frameworks and languages, such as ROS (Robot Operating System), C++, Tensorflow and Python.
* **Metodi Quantitativi per l'Informativa (Quantitative Methods for Computer Science)**: the project consisted in developing a few-shot-learning algorithm for image classification that was used in Duckietown initiative. Two methods have been developed, one simply using cross-entropy loss and a second one using triplet loss and metric learning. The dataset has been manually built from the Duckietown simulator in order to be able to train and test the two methods. This project was further extended to be used as my bachelor's thesis.
* **Bachelor's thesis**: the previous project was extended in order to make the algorithms faster so to be suitable for real-time application in Duckietown. Moreover, a datasets of real images was created by adapting an existing dataset and it was used to fine tune the models and make them usable in real world. Finally, the algorithms were adapted to be used with images containing multiple objects, by applying them to smaller patches. Comparisons between the classification time and the performance of the two developed methods applied respectively to the simulated and to the real dataset are present.

## Master's degree projects
This folder contains the projects realized during the Master's degree in Artificial Intelligence and Robotics at Sapienza University of Rome. The following list contains the names of the courses and a brief description of the projects:

* **Fundamentals of Computer Graphics**: three homeworks in which I implemented very simple raytracer, path tracer and volumetric path tracer using the library YoctoGL. Everything has been done using Visual Studio Code with C/C++.
* **Interactive Graphics**: two homeworks consisting in the implementation of basic tasks in computer graphics, such as rendering a scene, shadowing, variable lighting and animations with WebGL, and a final project realized in group with other colleagues, which consists in the implementation of a web version of the mobile game "Crossy road", using WebGL and the advanced library ThreeJS. It is possible to play the game at [this link](https://lucpol98.github.io/university_projects/Master%20Degree/Interactive%20Graphics/Project/main.html).
* **Machine Learning**: two homeworks in which we needed to solve two different classification tasks using respectively traditional ML approaches and neural networks and two reinforcement learning projects, in which we had to implement respectively a very simplified version of Pacman and [TRPO](https://arxiv.org/abs/1502.05477v5), tested on Mujoco’s [HalfCheetah-v2] (https://gym.openai.com/envs/HalfCheetah-v2/) environment.
* **Natural Language Processing**: two homeworks on Word-in-Context Disambiguation (WiCD) and on Word Sense Disambiguation (WSD) using [PyTorch](https://pytorch.org/).
* **Neural Networks**: project on Alzheimer's diagnosis using transfer learning to perform binary classification on MR images. Comparisons between the use of different pretrained networks are reported as well as the improvement determined by the use of entropy in order to select the most informative images for training.
* **Vision and Perception**: project consisting in the implementation and the extension of the paper [SinGAN: Learning a Generative Model from a Single Natural Image](https://arxiv.org/abs/1905.01164). In particular, the extension of the paper consists in the adaptation of the network to make it work with medical images.
* **Autonomous and Mobile Robotics**: implementation and comparison between the classic and primitive-based versions of RRT*. This project has been developed in an Ubuntu 18.04 environment using C++ and performing simulations in CoppeliaSim 4.0 EDU.
* **Elective in Artificial Intelligence 1**: developed a novel approach for brain MRI denoising in the k-space using a Conditional Generative Adversarial Network (CGAN).
* **Elective in Artificial Intelligence 2**: the purpose of this project is to develop a social and interactive robot for people’s entertainment. It allows to play a cooperative version of the Tower of Hanoi game, in which the user and the robot execute a move alternately. Many tools and languages were used to implement all the components. In particular, HTML and JavaScript together with web development tools, such as three.js and jQuery, were used to develop a web app to make the user make his moves. NAOqi was used to connect a Pepper robot to a server that sends it the commands to execute. Pepper SDK plugin for Android Studio was used to test all the functionalities on a simulated robot before performing them with the actual robot. Finally, a Python framework called AIPlan4EU has been exploited to execute the planning in order to make the robot able to correctly choose the best next move to make.
* **Deep Learning**: implementation of several approaches to address the visual question answering task. PyTorch together with several libraries, such as Scikit-learn, NLTK, and PIL were used.
