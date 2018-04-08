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

#include <opencv2/imgproc.hpp>

#include "cooperative_driving/vision_types.h"
#include "cooperative_driving/disable_ros_warnings_pre.h"
#include "cooperative_driving_vision/Moment.h"
#include "cooperative_driving/disable_ros_warnings_post.h"

namespace cooperative_driving
{
/** Respresentation of an geometrical Ellipse.
 */
struct Ellipse
{
  double x;      //<! X-coordinate of center.
  double y;      //<! Y-coordinate of center.
  double a;      //<! Semi major-axis.
  double b;      //<! Semi minor-axis.
  double alpha;  //<! Counter-clockwise rotation relative to x-axis.

  /** Draws this Ellipse onto the given image.
   *
   * /param[inout] img The image to render onto.
   * \param[in] color Color for rendering.
   */
  void draw(cv::InputOutputArray img, const Color &color) const
  {
    cv::ellipse(img, { (int)x, (int)y }, { (int)a, (int)b }, alpha, 0, 360, color, 3);
  }
};

/** A descriptor of an image region. The attributes correspond to different properties of the
 * region.
 *
 * \note: The current implementation uses a 'lazy-moment' i.e. it only stores the raw moments and
 * computes the derived moments (centralized and invariant) on demand. This is more efficient when
 * extending the moments is common (e.g. in feature extraction) but comes at a cost when
 * access to the derived moments is frequent.
 */
class Moment
{
public:
  /** Creates an empty Moment.
   */
  Moment() : m00_(0), m10_(0), m01_(0), m11_(0), m20_(0), m02_(0), m21_(0), m12_(0), m30_(0), m03_(0)
  {
  }

  /** Creates a Moment with the given attributes.
   */
  Moment(uint64_t m00, uint64_t m10, uint64_t m01, uint64_t m11, uint64_t m20, uint64_t m02, uint64_t m21, uint64_t m12,
         uint64_t m30, uint64_t m03)
    : m00_(m00), m10_(m10), m01_(m01), m11_(m11), m20_(m20), m02_(m02), m21_(m21), m12_(m12), m30_(m30), m03_(m03)
  {
  }

  /** Creates a Moment from a cooperative_driving_vision::Moment message.
   */
  Moment(const cooperative_driving_vision::Moment &moment_msg)
    : m00_(moment_msg.m00)
    , m10_(moment_msg.m10)
    , m01_(moment_msg.m01)
    , m11_(moment_msg.m11)
    , m20_(moment_msg.m20)
    , m02_(moment_msg.m02)
    , m21_(moment_msg.m21)
    , m12_(moment_msg.m12)
    , m30_(moment_msg.m30)
    , m03_(moment_msg.m03)
  {
  }

  /** (0, 0)th raw moment. */
  inline uint64_t m00() const
  {
    return m00_;
  }

  /** (1, 0)th raw moment. */
  inline uint64_t m10() const
  {
    return m10_;
  }

  /** (0, 1)th raw moment. */
  inline uint64_t m01() const
  {
    return m01_;
  }

  /** (1, 1)th raw moment. */
  inline uint64_t m11() const
  {
    return m11_;
  }

  /** (2, 0)th raw moment. */
  inline uint64_t m20() const
  {
    return m20_;
  }

  /** (0, 2)th raw moment. */
  inline uint64_t m02() const
  {
    return m02_;
  }

  /** (2, 1)th raw moment. */
  inline uint64_t m21() const
  {
    return m21_;
  }

  /** (1, 2)th raw moment. */
  inline uint64_t m12() const
  {
    return m12_;
  }

  /** (3, 0)th raw moment. */
  inline uint64_t m30() const
  {
    return m30_;
  }

  /** (0, 3)th raw moment. */
  inline uint64_t m03() const
  {
    return m03_;
  }

  /** (1, 1)th centralized moment. */
  inline float mu11() const
  {
    return (1.0f * m11()) - (m10() * m01() / m00());
  }

  /** (2, 0)th centralized moment, i.e. variance of x coordinates. */
  inline float mu20() const
  {
    return (1.0f * m20()) - (m10() * xavg());
  }

  /** (0, 2)th centralized moment, i.e. variance of y coordinates. */
  inline float mu02() const
  {
    return (1.0f * m02()) - (m01() * yavg());
  }

  /** (2, 1)th centralized moment. */
  inline float mu21() const
  {
    return (1.0f * m21()) - (2 * xavg() * m11()) - (yavg() * m20()) + (2 * std::pow(xavg(), 2) * m01());
  }

  /** (1, 2)th centralized moment. */
  inline float mu12() const
  {
    return (1.0f * m12()) - (2 * yavg() * m11()) - (xavg() * m02()) + (2 * std::pow(yavg(), 2) * m10());
  }

  /** (3, 0)th centralized moment. */
  inline float mu30() const
  {
    return (1.0f * m30()) - (3 * xavg() * m20()) + (2 * std::pow(xavg(), 2) * m10());
  }

  /** (0, 3)th centralized moment. */
  inline float mu03() const
  {
    return (1.0f * m03()) - (3 * yavg() * m02()) + (2 * std::pow(yavg(), 2) * m01());
  }

  /**  //!< (1, 1)th scale invariant moment. */
  inline float n11() const
  {
    return mu11() / std::pow(m00(), 2);
  }

  /** (2, 0)th scale invariant moment. */
  inline float n20() const
  {
    return mu20() / std::pow(m00(), 2);
  }

  /** (0, 2)th scale invariant moment. */
  inline float n02() const
  {
    return mu02() / std::pow(m00(), 2);
  }

  /** (2, 0)th scale invariant moment. */
  inline float n21() const
  {
    return mu21() / std::pow(m00(), 2.5);
  }

  /** (0, 2)th scale invariant moment. */
  inline float n12() const
  {
    return mu12() / std::pow(m00(), 2.5);
  }

  /** (2, 0)th scale invariant moment. */
  inline float n30() const
  {
    return mu30() / std::pow(m00(), 2.5);
  }

  /** (0, 2)th scale invariant moment. */
  inline float n03() const
  {
    return mu03() / std::pow(m00(), 2.5);
  }

  inline std::array<float, 3> n() const
  {
    return { n11(), n20(), n02() };
  }

  inline float xavg() const
  {
    return m10() / m00();
  }

  inline float yavg() const
  {
    return m01() / m00();
  }

  /** Extends this Moment by the given one.
   */
  inline Moment &operator+=(const Moment &rhs)
  {
    m00_ += rhs.m00_;
    m10_ += rhs.m10_;
    m01_ += rhs.m01_;
    m11_ += rhs.m11_;
    m20_ += rhs.m20_;
    m02_ += rhs.m02_;
    m21_ += rhs.m21_;
    m12_ += rhs.m12_;
    m30_ += rhs.m30_;
    m03_ += rhs.m03_;
    return *this;
  }

  /** This Moment's centroid.
   */
  cv::Point2f center() const
  {
    return cv::Point(xavg(), yavg());
  }

  /** Converts this Moment to a cooperative_driving_vision::Moment-message.
   */
  operator cooperative_driving_vision::Moment() const
  {
    cooperative_driving_vision::Moment result;
    result.m00 = m00();
    result.m10 = m10();
    result.m01 = m01();
    result.m11 = m11();
    result.m20 = m20();
    result.m02 = m02();
    result.m21 = m21();
    result.m12 = m12();
    result.m30 = m30();
    result.m03 = m03();
    return result;
  }

  /** Converts this Moment to an Ellipse that has same Moment.
   *
   * The Ellipse can be used for a rough representation.
   */
  operator Ellipse() const
  {
    return Ellipse(
        { center().x, center().y,
          std::sqrt(2 * (mu20() + mu02() + std::sqrt(std::pow(mu20() - mu02(), 2) + 4 * std::pow(mu11(), 2))) / m00()),
          std::sqrt(2 * (mu20() + mu02() - std::sqrt(std::pow(mu20() - mu02(), 2) + 4 * std::pow(mu11(), 2))) / m00()),
          0.5f * (atan((2.0f * mu11()) / (mu20() - mu02()))) });
  }

  /** Calculates the combination of the given Moments.
   */
  friend Moment operator+(const Moment &lhs, const Moment &rhs)
  {
    Moment result(lhs);
    result += rhs;
    return result;
  }

  /** Converts this Moment to a human-readable representation.
   */
  friend std::ostream &operator<<(std::ostream &stream, const Moment &moment)
  {
    stream << "Moment(" << moment.m00() << ", " << moment.m10() << ", " << moment.m01() << ", " << moment.m11() << ", "
           << moment.m20() << ", " << moment.m02() << ")";
    return stream;
  }

  /** Draws an Ellipse with an equivalent Moment onto the given image.
   *
   * /param[inout] img The image to render onto.
   * \param[in] color Color for rendering.
   */
  void draw(cv::InputOutputArray img, const Color &color) const
  {
    static_cast<Ellipse>(*this).draw(img, color);
  }

private:
  uint64_t m00_;
  uint64_t m10_;
  uint64_t m01_;
  uint64_t m11_;
  uint64_t m20_;
  uint64_t m02_;
  uint64_t m21_;
  uint64_t m12_;
  uint64_t m30_;
  uint64_t m03_;
};

}  // namespace cooperative_driving
