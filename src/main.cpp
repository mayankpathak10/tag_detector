/*MIT License

Copyright (c) 2019 Mayank Pathak

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 *
 * @file main.cpp
 * @brief tag_detector package
 *
 * @section DESCRIPTION
 *
 *  This file is the main file to implement the publisher node.
 *
 * @dependencies: tag_detector.hpp.
 * @author Mayank Pathak
 * @version 1
 * @date 2019-03-04
 */

#include "../include/tag_detector.hpp"

int main(int argc, char **argv) {
  publisher_node publisher_node;  // Class object created

  ros::init(argc, argv, "pub_node");  // Node name
  ros::NodeHandle n;                  // Node Handle

  std::string video_source =
      "/home/shivang/catkin_ws/src/tag_detector/data/4.mp4";

  ROS_DEBUG("Video Source Path: [%s]", video_source.c_str());
  cv::VideoCapture cap(video_source);

  if (!cap.isOpened()) {  // Check for invalid input
    ROS_ERROR("Could not open or find the image");
    return -1;
  }

  int total_frames = cap.get(CV_CAP_PROP_FRAME_COUNT);
  ROS_INFO("Total frames in Video: %d", total_frames);

  if (!ros::ok()) ROS_FATAL_STREAM("ROS node not running...");

  ros::Rate loop_rate(5);
  for (int i = 1; i < total_frames - 1; ++i) {
    ROS_INFO("Reading Frame: %d , Gaussian Kernel Size: %d ", i, kernel_size);

    cv::Mat frame = publisher_node.readFrame(i, video_source);
    cv::Mat bw_image = publisher_node.apply_gaussian_blur(frame, kernel_size);

    ros::spinOnce();
  }

  loop_rate.sleep();
  cap.release();
}
