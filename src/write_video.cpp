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
 * @file write_video.cpp
 * @brief tag_detector package
 *
 * @section DESCRIPTION
 *
 *  This file is reads the frame saved from subscriber node and writes a video to results folder.
 *
 * @dependencies: tag_detector.hpp.
 * @author Mayank Pathak
 * @version 1
 * @date 2019-03-04
 */

#include "../include/tag_detector.hpp"

int main() { 
  std::string path = ros::package::getPath("tag_detector"); //Returns path of the package
  std::string frames_path = "/results/videos";
  std::string video_path = "/results/frames";
  std::cout << "Writing Video from frames at Path: " << path + filePath << std::endl;

  cv::VideoCapture in_capture(frames_path"/dark/%d.jpg");
  cv::Mat img;
  in_capture.read(img);
  std::cout << "Image Shape : " << img.size() << std::endl;
  auto frame_height = img.size().height;
  auto frame_width = img.size().width;

  cv::VideoWriter out_capture(video_path"/Out1video.avi",
                              CV_FOURCC('M', 'J', 'P', 'G'), 30,
                              cv::Size(frame_width, frame_height));

  while (in_capture.isOpened()) {
    in_capture >> img;
    if (img.empty()) break;

    out_capture.write(img);
  }

  std::cout << "Finished!!!";
  in_capture.release();
  out_capture.release();
  return 0;
}
