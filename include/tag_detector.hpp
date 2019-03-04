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
 * @file tag_detector.hpp
 * @brief tag_detector package
 *
 * @section DESCRIPTION
 *
 *  This file is a header file to declare all the class variables and
 *  functions that will be used for Apriltag detection.
 *
 * @dependencies: None.
 * @author Mayank Pathak
 * @version 1
 * @date 2019-03-04
 */

#ifndef INCLUDE_TAG_DETECTOR_HPP_
#define INCLUDE_TAG_DETECTOR_HPP_

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ctime>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <sstream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/String.h"
// #include "tag_detector/gaussian_size.h"

/**
 * @brief      Class for publisher node.
 */
class publisher_node {
 private:
  /**
   * { variable to store input image frame }
   */
  cv::Mat input_image;

 public:
  /**
   * @brief      Reads a frame.
   *
   * @param[in]  { frame number to read }
   * @param[in]  { path of image to read }
   *
   * @return     { image frame }
   */
  cv::Mat readFrame(int, std::string);

  /**
   * @brief      { Applies gaussian blur to image }
   *
   * @param[in]  { image for blurring }
   * @param[in]  { kernel size }
   *
   * @return     { Blurred Image }
   */
  cv::Mat apply_gaussian_blur(cv::Mat, int);
};

/**
 * @brief      Class for subscriber node.
 */
class subscriber_node {
 private:
  cv::Mat sub_image;
  cv::Mat mask;
  cv::Mat eq_img;
  cv_bridge::CvImagePtr cv_ptr;

 public:
  /**
   * @brief      Calculates the PSF.
   *
   * @param      outputImg   The output image
   * @param[in]  filterSize  The filter size
   * @param[in]  R           { radius for filter circle }
   *
   * @return     Image containing filter.
   */
  cv::Mat calcPSF(cv::Mat& outputImg, cv::Size filterSize, int R);  // NOLINT

  /**
   * @brief      { Calculates FFT shift }
   *
   * @param[in]  inputImg   The input image
   * @param      outputImg  The output image
   *
   * @return     { Image containing shift as pixel values }
   */
  cv::Mat fftshift(const cv::Mat& inputImg, cv::Mat& outputImg);  // NOLINT

  /**
   * @brief      { Applies 2D filter }
   *
   * @param[in]  inputImg   The input image
   * @param      outputImg  The output image
   * @param[in]  H          { filter size }
   *
   * @return     { Filtered Image }
   */
  cv::Mat filter2DFreq(const cv::Mat& inputImg, cv::Mat& outputImg,  // NOLINT
                       const cv::Mat& H);

  /**
   * @brief      Calculates the wnr filter.
   *
   * @param[in]  input_h_PSF  The input h psf
   * @param      output_G     The output g
   * @param[in]  nsr          The nsr
   *
   * @return     The wnr filter.
   */
  cv::Mat calcWnrFilter(const cv::Mat& input_h_PSF,
                        cv::Mat& output_G,  // NOLINT
                        double nsr);

  /**
   * @brief      { To calculate angle }
   *
   * @param[in]   { Point1 }
   * @param[in]   { Point2 }
   * @param[in]   { Ref Point }
   *
   * @return     { angle }
   */
  double angle(cv::Point, cv::Point, cv::Point);

  /**
   * @brief      { finds squares in image }
   *
   * @param[in]  image    The image
   * @param      squares  The squares matrix (empty)
   *
   * @return     { squares  }
   */
  std::vector<std::vector<cv::Point> > findSquares(
     const cv::Mat& image, std::vector<std::vector<cv::Point>>& squares);

  /**
   * @brief      Draws squares.
   *
   * @param      image    The image
   * @param[in]  squares  The squares
   *
   * @return     { image with sqaures}
   */
  cv::Mat drawSquares(
    cv::Mat& image, const std::vector<std::vector<cv::Point> >& squares);
};

#endif  // INCLUDE_TAG_DETECTOR_HPP_
