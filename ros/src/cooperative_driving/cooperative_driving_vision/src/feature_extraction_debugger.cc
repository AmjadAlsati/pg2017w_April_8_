//////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, CCS Labs
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////////
#include <iomanip>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cooperative_driving/feature_extractor.h"
#include "cooperative_driving/vision_types.h"

using namespace cooperative_driving;

/** Converts the given color from HSV to BGR
 *
 * \param[in] src Color in HSV
 *
 * \return The converted BGR color
 */
inline Color convert_color(Color src)
{
  cv::Mat srcMat(1, 1, CV_8UC3);
  srcMat.at<cv::Vec3b>(0, 0) = src;

  cv::Mat destMat(1, 1, CV_8UC3);
  cv::cvtColor(srcMat, destMat, cv::COLOR_HSV2BGR);

  return destMat.at<Color>(0, 0);
}

/** Handles the current image and draws all detected features in it
 *
 * This function will all the feature extractor to extract features which then will
 * be dran above the given image.
 *
 * \param[in] color_ranges The defined color ranges in which to look for blobs
 * \param[in] img Reference to the current image
 *
 */
void handle_image(const std::vector<ColorRange> &color_ranges, std::vector<ColorPattern> /*color_patterns*/,
                  cv::Mat &img)
{
  const auto detected_regions = extract_regions(color_ranges, img, 4, 150);
  const auto line = extract_line(img, 0.0, 5, 60);

  // draw lines
  if (line >= 0)
  {
    cv::line(img, { line, img.rows - 1 }, { line, img.rows - 20 }, { 255, 00, 0 }, 2);
    cv::putText(img, "x=" + std::to_string(line), { line + 5, img.rows - 10 }, 0, 0.5, { 255, 00, 0 }, 1, CV_AA);
  }
  // draw regions
  for (const auto region : detected_regions)
  {
    region.draw(img, true);

    const auto &moment = region.moment;
    std::stringstream text;
    text << std::fixed << std::setprecision(0) << "(" << moment.center().x << "," << moment.center().y << ")";
    cv::putText(img, text.str(), moment.center(), 0, 0.3, { 255, 255, 255 }, 1, CV_AA);
  }
}

/** The main function of the image debugger.
 *
 * Color range are defined here for extracting blob features. Also the image
 * is processed and shortcuts are handled.
 *
 * \param[in] argc The number of arguments passed to the program
 * \param[in] argv The argument values of passed arguments
 */
int main(int argc, const char *argv[])
{
  const ColorRange red = std::make_pair(Color(70, 0, 140), Color(104, 78, 250));
  const ColorRange green = std::make_pair(Color(113, 114, 28), Color(255, 230, 72));
  const ColorRange orange = std::make_pair(Color(0, 0, 130), Color(75, 255, 255));
  const ColorRange blue = std::make_pair(Color(84, 0, 24), Color(126, 100, 74));
  std::vector<ColorRange> color_ranges{ red, green, orange, blue };
  // std::vector<ColorRange> color_ranges{red, green};
  // std::vector<ColorRange> color_ranges{orange, blue};
  std::vector<ColorPattern> color_patterns{ { orange, blue, orange } };

  if (argc == 2)
  {
    cv::Mat image;
    image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    if (!image.data)
    {
      std::cout << "Could not open or find the image" << std::endl;
      return -1;
    }
    handle_image(color_ranges, color_patterns, image);

    while (true)
    {
      cv::imshow("Feature extractor", image);
      switch (cv::waitKey(1))
      {
        case 'q':
          return 0;
        case 's':
          cv::imwrite("test_static.png", image);
          break;
      }
    }
  }
  else
  {
    cv::VideoCapture stream1(0);
    auto snapshot_count = 0;

    if (!stream1.isOpened())
    {
      std::cout << "cannot open camera";
    }

    cv::Mat cameraFrame;
    stream1.read(cameraFrame);
    // unconditional loop
    while (true)
    {
      cv::Mat cameraFrame;
      cv::Mat3b backupCameraFrame;
      stream1.read(cameraFrame);
      cameraFrame.copyTo(backupCameraFrame);
      handle_image(color_ranges, color_patterns, cameraFrame);
      cv::imshow("cam", cameraFrame);
      switch (cv::waitKey(1))
      {
        case 'q':
          return 0;
        case 's':
          std::stringstream s;
          s << "snapshot_" << snapshot_count << ".png";
          snapshot_count += 1;
          cv::imwrite(s.str(), backupCameraFrame);
      }
    }
  }
  return 0;
}
