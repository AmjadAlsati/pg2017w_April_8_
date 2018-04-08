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
#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <numeric>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "cooperative_driving/region.h"
#include "cooperative_driving/vision_types.h"

namespace cooperative_driving
{
/** Represent an extracted line feature
 */
struct LineFeature
{
  float location;
};

/** Computes Regions for contiguous areas matching a given ColorRange in the given image.
  *
  * Uses line regions that were extracted before to combine them into larger
  * regions.
  *
  * \param[in] color_ranges The defined color ranges to look for blobs
  * \param[in] frame The current image frame that is processed
  * \param[in] min_line_length The minimum length at which lines are used
  * \param[in] min_size The minimum size at which blobs are extracted
  *
  * \return A vector of extracted regions
 */
std::vector<Region> extract_regions(const std::vector<ColorRange> &color_ranges, const cv::Mat &image,
                                    const int min_line_length = 2, const unsigned min_size = 150);

/** Detects the horizontal location of a bright line at the given height.
 *
 * To reduce impact of noise, the extraction uses an average of multiple image rows.
 *
 * \param image Image containing the line.
 * \param height Normalized vertical location. This must be in the interval [0,1].
 * \param neighborhood Number of rows averaged for extraction.
 * \param lower_canny_threshold The lower threshold used for the canny filter. Max. threshold is 3 time the lower
 * threshold.
 *
 * \return The position of the extracted line
 */
int extract_vertical_line(const cv::Mat &image, const float height, const int neighborhood, const int lower_canny_threshold);

/** Detects the vertical location of a bright line at the given height.
 *
 * To reduce impact of noise, the extraction uses an average of multiple image rows.
 *
 * \param image Image containing the line.
 * \param height Normalized vertical location. This must be in the interval [0,1].
 * \param neighborhood Number of rows averaged for extraction.
 * \param lower_canny_threshold The lower threshold used for the canny filter. Max. threshold is 3 time the lower
 * threshold.
 *
 * \return The position of the extracted line
 */
int extract_horizontal_line(const cv::Mat &image, const float height, const int neighborhood, const int canny_threshold);
}
