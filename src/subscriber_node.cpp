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

// Pointer to convert image from ROS format 
cv_bridge::CvImagePtr cv_ptr;
cv::Mat sub_image;
cv::Mat mask;
cv::Mat eq_img;
int R = 2;  // Radius for Wiener Filer
int snr = 3000;  // SNR for FFT
int z = 1;  // index to save images
subscriber_node subscriber_node;  // Class Object


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
    cv::imshow("Threshold Image", gray);
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
          fabs(contourArea(cv::Mat(approx))) < 10000 &&
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

cv::Mat subscriber_node::calcPSF(cv::Mat& outputImg, cv::Size filterSize,
                                 int R) {
  cv::Mat h(filterSize, CV_32F, cv::Scalar(0));
  cv::Point point(filterSize.width / 2, filterSize.height / 2);
  cv::circle(h, point, R, 255, -1, 8);
  cv::Scalar summa = sum(h);
  outputImg = h / summa[0];

  return outputImg;
}

cv::Mat subscriber_node::fftshift(const cv::Mat& inputImg, cv::Mat& outputImg) {
  outputImg = inputImg.clone();
  int cx = outputImg.cols / 2;
  int cy = outputImg.rows / 2;
  cv::Mat q0(outputImg, cv::Rect(0, 0, cx, cy));
  cv::Mat q1(outputImg, cv::Rect(cx, 0, cx, cy));
  cv::Mat q2(outputImg, cv::Rect(0, cy, cx, cy));
  cv::Mat q3(outputImg, cv::Rect(cx, cy, cx, cy));
  cv::Mat tmp;
  q0.copyTo(tmp);
  q3.copyTo(q0);
  tmp.copyTo(q3);
  q1.copyTo(tmp);
  q2.copyTo(q1);
  tmp.copyTo(q2);

  return outputImg;
}

cv::Mat subscriber_node::filter2DFreq(const cv::Mat& inputImg,
                                      cv::Mat& outputImg, const cv::Mat& H) {
  cv::Mat planes[2] = {cv::Mat_<float>(inputImg.clone()),
                       cv::Mat::zeros(inputImg.size(), CV_32F)};
  cv::Mat complexI;
  cv::merge(planes, 2, complexI);
  cv::dft(complexI, complexI, cv::DFT_SCALE);
  cv::Mat planesH[2] = {cv::Mat_<float>(H.clone()),
                        cv::Mat::zeros(H.size(), CV_32F)};
  cv::Mat complexH;
  cv::merge(planesH, 2, complexH);
  cv::Mat complexIH;
  cv::mulSpectrums(complexI, complexH, complexIH, 0);
  cv::idft(complexIH, complexIH);
  cv::split(complexIH, planes);
  outputImg = planes[0];

  return outputImg;
}

cv::Mat subscriber_node::calcWnrFilter(const cv::Mat& input_h_PSF,
                                       cv::Mat& output_G, double nsr) {
  cv::Mat h_PSF_shifted;
  subscriber_node::fftshift(input_h_PSF, h_PSF_shifted);
  cv::Mat planes[2] = {cv::Mat_<float>(h_PSF_shifted.clone()),
                       cv::Mat::zeros(h_PSF_shifted.size(), CV_32F)};
  cv::Mat complexI;
  cv::merge(planes, 2, complexI);
  cv::dft(complexI, complexI);
  cv::split(complexI, planes);
  cv::Mat denom;
  pow(abs(planes[0]), 2, denom);
  denom += nsr;
  cv::divide(planes[0], denom, output_G);

  return output_G;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  std::vector<std::vector<cv::Point> > squares;
  ROS_DEBUG("Came into imageCallback");
  try {
    // time clock variable
    std::clock_t c_start = std::clock();

    cv_bridge::toCvShare(msg, "mono8")->image;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    cv_ptr->image.copyTo(sub_image);

    cv::Mat Copy2;
    cv::cvtColor(sub_image, Copy2, cv::COLOR_GRAY2BGR);
    cv::Mat imgIn = sub_image;
    cv::Mat imgOut;

    // it needs to process even image only
    cv::Rect roi = cv::Rect(0, 0, imgIn.cols & -2, imgIn.rows & -2);
    // Hw calculation (start)
    cv::Mat Hw, h;
    cv::Mat filter = subscriber_node.calcPSF(h, roi.size(), R);
    imgOut = subscriber_node.calcWnrFilter(filter, Hw, 1.0 / double(snr));
    // Hw calculation (stop)
    // filtering (start)
    imgOut = subscriber_node.filter2DFreq(imgIn(roi), imgOut, Hw);
    // filtering (stop)
    imgOut.convertTo(imgOut, CV_8U);
    cv::normalize(imgOut, imgOut, 0, 255, cv::NORM_MINMAX);
    // imwrite("result.jpg", imgOut);

    // cv::Mat eq_img;
    cv::equalizeHist(imgOut, ::eq_img);

    // compute mask (you could use a simple threshold if the image is always as
    // good as the one you provided) cv::threshold(eq_img, mask, 10 , 255,
    // CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
    cv::adaptiveThreshold(eq_img, mask, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          CV_THRESH_BINARY_INV, 31, 0);

    cv::cvtColor(imgOut, imgOut, cv::COLOR_GRAY2BGR);
    // cv::imshow("view",imgOut);
    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
    cv::cvtColor(::eq_img, ::eq_img, cv::COLOR_GRAY2BGR);

    squares = subscriber_node.findSquares(mask, squares);
    cv::Mat Output = subscriber_node.drawSquares(Copy2, squares);

    cv::imshow("Final Output", Output);
    auto s = std::to_string(z);

    std::string path = ros::package::getPath("tag_detector"); //Returns path of the package
    std::string file_path = "/results/frames/";
    std::string complete_path = path + file_path;

    cv::imwrite(complete_path + s + ".jpg", Output);
    z += 1;
    cv::waitKey(1);

    std::clock_t c_end = std::clock();

    double time_elapsed_ms = 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
    ROS_INFO("CPU Computation Time per Frame: %f %s", time_elapsed_ms, "ms");
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "subs_node");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  ROS_INFO("before imageCallback");

  while (nh.ok()) {
  image_transport::Subscriber sub =
      it.subscribe("/image_raw", 1, imageCallback);
  ROS_DEBUG("Came Out of imageCallback");
  ros::spin();
  cv::destroyAllWindows();
}
cv::destroyAllWindows();
}
