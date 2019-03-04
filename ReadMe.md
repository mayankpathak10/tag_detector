[![Build Status](https://travis-ci.com/mayankpathak10/tag_detector.svg?branch=maste)](https://travis-ci.com/mayankpathak10/tag_detector) [![Coverage Status](https://coveralls.io/repos/github/mayankpathak10/tag_detector/badge.svg?branch=master)](https://coveralls.io/github/mayankpathak10/tag_detector?branch=master) [![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)


## Overview:
The main idea of this project is to implement ROS Concepts like publisher, subscriber, services and using them to detect Apriltags from a given video. 

A node is created to publish the image frames using a topic ```/image_raw ``` which is subscribed by another node and is used for further operations on the image. 

A service, ```/set_blue_window_size``` is added to the publisher node to change the Gaussian filter kernel window size. 

The subscriber node also evaluates the sharpness of image using FFT and deblurs the image if required. 
This node also shows the computational time required to process each frame. The detected Apriltags are highlighted by green rectangles around them. The frames are then saved to the package location.

It also includes a program to write video from the image frames saved in the package results.


## About Author

__Mayank Pathak:__ Graduate Student in M.Eng., Robotics, working as Teaching Assistant for Grad. Level Robotics Course for Machine Learning at the University of Maryland, College Park. I am interested to pursue career in the field of Computer Vision, Machine Learning and related fields.

## Project Approach:
1. Publisher Node is created.
	A video recorded from the printout of Apriltags is loaded, frames extracted and then each frame is publised in order.  
2. Subscriber Node is created.
	To detect the April tags, various manipulation operations are applied and output is shown.
3. ROS service server is then implemented in Publisher Node.
4. Fast Fourier Transform (FFT) is then used to Deblur the image.
6. Quality is assured by delivering Unit Tests for overall source code and full coverage in coveralls. 

The UML diagrams for the project are as follows:
### UML Activity Diagram
![activity_diag](https://github.com/mayankpathak10/tag_detector/blob/master/UML/initial/tag_detector_ActivityDiagram.jpeg)
### UML Class Diagram
![class_diag](https://github.com/mayankpathak10/tag_detector/blob/master/UML/initial/tag_detector_Class_diagram.jpeg)
