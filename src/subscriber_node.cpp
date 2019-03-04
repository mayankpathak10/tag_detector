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
 * @file subscriber_node.cpp
 * @brief tag_detector package
 *
 * @section DESCRIPTION
 *
 *  This file is the main file to implement the subscriber node.
 *
 * @dependencies: tag_detector.hpp.
 * @author Mayank Pathak
 * @version 1
 * @date 2019-03-04
 */

#include "../include/tag_detector.hpp"

cv::Mat subscriber_node::calcPSF(cv::Mat& outputImg, cv::Size filterSize,
                                 int R) {}
cv::Mat subscriber_node::fftshift(const cv::Mat& inputImg, cv::Mat& outputImg) {
}
cv::Mat subscriber_node::filter2DFreq(const cv::Mat& inputImg,
                                      cv::Mat& outputImg, const cv::Mat& H) {}
cv::Mat subscriber_node::calcWnrFilter(const cv::Mat& input_h_PSF,
                                       cv::Mat& output_G, double nsr) {}
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {}

int main(int argc, char** argv) {
  ros::init(argc, argv, "subs_node");
  ros::NodeHandle nh;

  ros::spin();
  cv::destroyAllWindows();
}
