[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)


## Overview:
The main idea of this project is to implement ROS Concepts like publisher, subscriber, services and using them to detect Apriltags from a given video. 

A node is created to publish the image frames using a topic ```/image_raw ``` which is subscribed by another node and is used for further operations on the image. 

A service, ```/set_blue_window_size``` is added to the publisher node to change the Gaussian filter kernel window size. 

The subscriber node also evaluates the sharpness of image using FFT and deblurs the image if required. 
This node also shows the computational time required to process each frame. The detected Apriltags are highlighted by green rectangles around them. The frames are then saved to the package location.

It also includes a program to write video from the image frames saved in the package results.


## About Author
```
__Mayank Pathak:__ Graduate Student in M.Eng., Robotics, working as Teaching Assistant for Grad. Level Robotics Course for Machine Learning at the University of Maryland, College Park. I am interested to pursue career in the field of Computer Vision, Machine Learning and related fields.
```
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


## Deveopment Using Solo Iterative Process (SIP)
In development of this software module Solo Interative Process (SIP) was followed. Using SIP this software module is divided into two parts: the product backlog, and code of the software.

First, the product backlog was developed as per the requirements. From these entries, highest priority requirements were selected for the first iteration. In the product backlog, estimated time of completion was allotted to every task. Actual completion time was compared with estimated completion time and based on that, time allotment for future tasks was modified.

After the planning was done, based on UML class diagram, stub classes were written followed by implementation and Documentation.

Following is the link to the spreadsheet that contains detailed entries of the product backlog and error log:[Product Backlog](https://docs.google.com/spreadsheets/d/1IvdiBzc3T5-pWsjkV7SkqSRE5-VnDC9NLZS4jSi_JfM/edit?usp=sharing)


## Dependencies
- Ubuntu 16.04
- ROS Kinetic
- Catkin
- OpenCV 3.4.1
- Packages included in ROS Kinetic:	
    - roscpp
	- std_msgs
	- genmsg
	- geometry_msgs
	- cv_bridge


## Sample Images
An image frame from input, after applying adaptive threshold and the resulting frame are shown below:

![input_image](https://github.com/mayankpathak10/tag_detector/blob/video_write/results/input_frame.jpg)
![thresh_image](https://github.com/mayankpathak10/tag_detector/blob/video_write/results/light_thresh.png)
![Output_image](https://github.com/mayankpathak10/tag_detector/blob/video_write/results/frames/light/20.jpg)

## How to build
If you do not have a catkin workspace, in a new terminal:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/mayankpathak10/tag_detector
cd ..
catkin_make
```
If you wish to run this code in an existing catkin workspace:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/
git clone https://github.com/mayankpathak10/tag_detector
cd ..
catkin_make
```

## Demo

After following the above build instructions:

first run rosmaster from a terminal:
```
roscore
```
To run the demo, in a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
rosrun tag_detector pub_node
```
This will run the publisher node which will read Normal lighting condition video. To change the video input, simply give argument with the above command as follows:

```rosrun tag_detector pub_node 1```

Argument `1` refers to light video and is Default. Argument `2` will run publisher with Dark lighting video.

Now to subscriber Node, in a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
rosrun tag_detector subs_node 
```
This will pop up two windows showing output frames.

### Saving Video
 To save video from the frames generated, in a new terminal:
```
rosrun tag_detector write_video
```

## Doxygen Documentation
To generate Doxygen Documentation in HTML and LaTEX, follow the next steps:
```
cd <path to repository>
mkdir <documentation_folder_name>
cd <documentation_folder_name>
doxygen -g <config_file_name>
```
Inside the configuration file, update:
```
PROJECT_NAME = 'project name'
INPUT = ../agv_navigation ../include ../test
```
Doxygen files will be generated to /docs folder

To view them in a browser:
```
cd docs
cd html
firefox index.html
```
## Known issues and Bugs
* ROS node needs to be terminated using `Ctrl+C` after all video frames are read.

