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
 * @file publisher_node.cpp
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

/**
 * @brief      Reads a frame.
 *
 * @param[in]  frame_number  The frame number
 * @param[in]  video_source  The video source
 *
 * @return     { frame }
 */
cv::Mat publisher_node::readFrame(int frame_number, std::string video_source) {
  cv::Mat frame;  // matrix to store input frame
  // Video Object to read frame
  cv::VideoCapture cap(video_source);
  ROS_DEBUG("Reading Frame #%d", frame_number);
  cap.set(cv::CAP_PROP_POS_FRAMES, frame_number);
  cap >> frame;
  cv::resize(frame, frame, cv::Size(), 0.50, 0.50);
  cap.release();
  return frame;
}

/**
 * @brief      { Applies gaussian blur to image }
 *
 * @param[in]  { image for blurring }
 * @param[in]  { kernel size }
 *
 * @return     { Blurred Image }
 */
cv::Mat publisher_node::apply_gaussian_blur(cv::Mat image, int kernel_size) {
  cv::Mat bw_image;
  cv::cvtColor(image, bw_image, cv::COLOR_BGR2GRAY);
  cv::Mat blur_image;
  cv::GaussianBlur(bw_image, blur_image, cv::Size(kernel_size, kernel_size), 0,
                   0);
  return blur_image;
}
