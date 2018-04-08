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

#include <algorithm>
#include <utility>
#include <vector>

#include <geometry_msgs/Point.h>

#include "cooperative_driving/util.h"
#include "cooperative_driving/vision_types.h"

namespace cooperative_driving
{
/**
 * Check if specified input vector is within the color range.
 *
 * \param input The input vector in BGR format to check whether it is within the color range.
 * \param color_range The color range to check if the input vector is within that range.
 * \return True if input vector is in range.
 */
inline bool checkInRange(const Color &input, const ColorRange &color_range)
{
  return color_range.first[0] <= input[0] && color_range.second[0] >= input[0] && color_range.first[1] <= input[1] &&
         color_range.second[1] >= input[1] && color_range.first[2] <= input[2] && color_range.second[2] >= input[2];
}

/**
 * Check if specified vector is within the specified range of colors.
 *
 * \param color                     The input color vector in BGR format.
 * \param color_ranges              Color ranges to check whether the input vector is within range.
 * \param color_range_in_range[out] if vector is within a color range, this parameter is set to the corresponding color
 * range.
 * \return                          Iterator to the first color range in which the input color lies.
 */
inline std::vector<ColorRange>::const_iterator findMatchingRange(const Color &color,
                                                                 const std::vector<ColorRange> &color_ranges)
{
  return std::find_if(color_ranges.begin(), color_ranges.end(),
                      [&color](const ColorRange &color_range) { return checkInRange(color, color_range); });
}

/**
 * Computes the interpolated x-location at the specified y-location on the line described by the support-points.
 *
 * \param line_row_location The points y-location
 * \param lines The support points
 */
inline float line_location(const int line_row_location, const std::vector<geometry_msgs::Point> &lines)
{
  if (line_row_location > lines.front().y)
  {
    return lines.front().x;
  }
  else
  {
    for (auto it = std::make_pair(lines.begin(), std::next(lines.begin())); it.second != lines.end();
         it.first++, it.second++)
    {
      if (line_row_location > it.second->y)
      {
        return scale_to_range(line_row_location, { it.first->y, it.second->y }, { it.first->x, it.second->x });
      }
    }
  }
  return lines.back().x;  // lookup-location higher than last element
}

/** Compute the gauß-sum of n. */
template <typename T>
inline T gauss_sum(T n)
{
  return (n * n + n) / 2;
}

/** Compute the square-pyramidal-number. */
template <typename T>
inline T square_pyramidal_number(T n)
{
  return (2 * n * n * n + 3 * n * n + n) / 6;
}

}  // namespace cooperative_driving
