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

#include "cooperative_driving/disable_ros_warnings_pre.h"
#include "cooperative_driving/moment.h"
#include "cooperative_driving/disable_ros_warnings_pre.h"
#include "cooperative_driving_vision/Region.h"
#include "cooperative_driving/disable_ros_warnings_post.h"

namespace cooperative_driving
{
/**
 * Represents a colored region inside an image.
 */
struct Region
{
  Moment moment;       //!< Descriptor of the region's pixel-coordinates.
  ColorSum color_sum;  //!< Sum of the Colors of all represented pixels.

  /**
   * Creates a Moment from the given values.
   */
  Region() : moment(), color_sum(0, 0, 0)
  {
  }

  /**
   * Copy-constructor.
   */
  Region(const Region &other) : moment(other.moment), color_sum(other.color_sum)
  {
  }

  Region(const cooperative_driving_vision::Region &region_message)
    : moment(region_message.moment)
    , color_sum(ColorSum(region_message.color.r, region_message.color.g, region_message.color.b) * float(moment.m00()))
  {
  }

  /**
   * Average color of a pixel represented by this Region.
   */
  Color average_color() const
  {
    return Color(color_sum / static_cast<int>(moment.m00()));
  }

  /**
   * Extends this Region by the given one.
   */
  inline Region &operator+=(const Region &rhs)
  {
    color_sum += rhs.color_sum;
    moment += rhs.moment;
    return *this;
  }

  /**
   * Returns a combined Region of both this and the given one.
   */
  inline Region operator+(const Region &rhs)
  {
    Region result(*this);
    result += rhs;
    return result;
  }

  /**
   * Convert the Region to a cooperative_driving_vision::Region message
   */
  operator cooperative_driving_vision::Region() const
  {
    cooperative_driving_vision::Region result;
    result.color.r = average_color()[0];
    result.color.g = average_color()[1];
    result.color.b = average_color()[2];
    result.moment = moment;
    return result;
  }

  /**
   * Draws the Region.
   *
   * Draws the Region as an ellipsoid of the average-color representing the Region's Moment onto the given image.
   *
   * \param[inout] img The image to draw onto.
   * \param[in] negative Use the average color's negative.
   */
  void draw(cv::InputOutputArray img, bool negative = false) const
  {
    Color color(average_color());
    if (negative)
    {
      cv::bitwise_not(average_color(), color);
    }
    moment.draw(img, color);
  }
};

}  // namespace cooperative_driving
