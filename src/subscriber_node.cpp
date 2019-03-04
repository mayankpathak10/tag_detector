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

// cv::Mat subscriber_node::calcPSF(cv::Mat& outputImg, cv::Size filterSize,
//                                  int R) {}
// cv::Mat subscriber_node::fftshift(const cv::Mat& inputImg, cv::Mat&
// outputImg) {
// }
// cv::Mat subscriber_node::filter2DFreq(const cv::Mat& inputImg,
//                                       cv::Mat& outputImg, const cv::Mat& H)
//                                       {}
// cv::Mat subscriber_node::calcWnrFilter(const cv::Mat& input_h_PSF,
//                                        cv::Mat& output_G, double nsr) {}

cv_bridge::CvImagePtr cv_ptr;
cv::Mat sub_image;
cv::Mat mask;
cv::Mat eq_img;
subscriber_node subscriber_node;
double subscriber_node::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1 * dx2 + dy1 * dy2) /
         sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

std::vector<std::vector<cv::Point> > subscriber_node::findSquares(
    const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares) {
  squares.clear();

  cv::Mat timg(image);
  cv::medianBlur(image, timg, 3);
  cv::Mat gray0(timg.size(), CV_8U), gray;

  std::vector<std::vector<cv::Point> > contours;

  for (int c = 0; c < 3; c++) {
    int ch[] = {c, 0};
    cv::mixChannels(&timg, 1, &gray0, 1, ch, 1);

    cv::adaptiveThreshold(gray0, gray, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          CV_THRESH_BINARY_INV, 31, 0);
    cv::dilate(gray, gray, cv::Mat(), cv::Point(-1, -1));

    // find contours and store them all as a list
    cv::Mat gray2;
    cv::cvtColor(gray, gray2, cv::COLOR_GRAY2BGR);
    cv::imshow("view", gray);
    cv::findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> approx;

    // test each contour
    for (size_t i = 0; i < contours.size(); i++) {
      // approximate contour with accuracy proportional
      // to the contour perimeter
      cv::approxPolyDP(cv::Mat(contours[i]), approx,
                       cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);

      // square contours should have 4 vertices after approximation
      // relatively large area (to filter out noisy contours)
      // and be convex.
      // Note: absolute value of an area is used because
      // area may be positive or negative - in accordance with the
      // contour orientation
      if (approx.size() == 4 && fabs(contourArea(cv::Mat(approx))) > 700 &&
          fabs(contourArea(cv::Mat(approx))) < 4000 &&
          cv::isContourConvex(cv::Mat(approx))) {
        double maxCosine = 0;

        for (int j = 2; j < 5; j++) {
          // find the maximum cosine of the angle between joint edges
          double cosine = fabs(subscriber_node::angle(
              approx[j % 4], approx[j - 2], approx[j - 1]));
          maxCosine = MAX(maxCosine, cosine);
        }

        // if cosines of all angles are small
        // (all angles are ~90 degree) then write quandrange
        // vertices to resultant sequence
        if (maxCosine < 0.3) squares.push_back(approx);
      }
    }
    // }
  }
  return squares;
}

// the function draws all the squares in the image
cv::Mat subscriber_node::drawSquares(
    cv::Mat& image, const std::vector<std::vector<cv::Point> >& squares) {
  for (size_t i = 0; i < squares.size(); i++) {
    const cv::Point* p = &squares[i][0];

    int n = (int)squares[i].size();
    // dont detect the border
    if (p->x > 3 && p->y > 3)
      cv::polylines(image, &p, &n, 1, true, cv::Scalar(0, 255, 0), 1,
                    cv::LINE_AA);
  }

  // cv::imshow("view2", image);
  return image;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  std::vector<std::vector<cv::Point> > squares;
  ROS_DEBUG("Came into imageCallback");
  try {
    cv_bridge::toCvShare(msg, "mono8")->image;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    // sub_image = cv_ptr->image.clone();
    cv_ptr->image.copyTo(sub_image);

    cv::Mat Copy2;
    cv::cvtColor(sub_image, Copy2, cv::COLOR_GRAY2BGR);


    squares = subscriber_node.findSquares(Copy2, squares);
    cv::Mat Output = subscriber_node.drawSquares(Copy2, squares);

    cv::imshow("view2", Output);
    cv::waitKey(20);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "subs_node");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  ROS_INFO("before imageCallback");

  image_transport::Subscriber sub =
      it.subscribe("/image_raw", 1, imageCallback);
  ROS_INFO("Came Out of imageCallback");
  ros::spin();
  cv::destroyAllWindows();
}
