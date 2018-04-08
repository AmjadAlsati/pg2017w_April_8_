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

#include <cmath>
#include <functional>
#include <numeric>
#include <type_traits>

namespace cooperative_driving
{
/** Computes pi. */
constexpr double pi()
{
  return std::atan(1) * 4;
}

/** Computes the p-norm, a generalization of the euclidean distance (2-norm).  */
template <typename T, size_t N>
inline T p_norm(const std::array<T, N> &xs, const T p)
{
  static_assert(std::is_floating_point<T>::value, "Floating point required.");
  return std::pow(
      std::accumulate(xs.begin(), xs.end(), .0f, [&p](const T &sum, const T &x) { return sum + std::pow(x, p); }),
      1 / p);
}

/** Subtracts two arrays, element wise. */
template <typename T, size_t N>
inline std::array<T, N> operator-(const std::array<T, N> &lhs, const std::array<T, N> &rhs)
{
  static_assert(std::is_floating_point<T>::value, "Floating point required.");
  std::array<T, N> result;
  for (size_t i = 0; i < N; i++)
  {
    result[i] = lhs[i] - rhs[i];
  }
  return result;
}

/** Calculates the distance between two given 2-dimensional points.
 *
 * param[in] p1 The first input point
 * param[in] p2 The second input point
 *
 * \return The distance between the two given points
 */
inline float getDistance(cv::Point2f p1, cv::Point2f p2)
{
  return std::sqrt(std::pow(p2.y - p1.y, 2) + std::pow(p2.x - p1.x, 2));
}

/** Converts degrees to radian.
 *
 * \param[in] degree The angle in degree
 *
 * \return The converted value
 */
template <typename T>
constexpr T degree_to_radian(T degree)
{
  static_assert(std::is_floating_point<T>::value, "Floating point required.");
  return degree * pi() / 180;
}

/** Converts radians to degree.
 *
 * \param[in] degree The angle in radians
 *
 * \return The converted value
 */
template <typename T>
constexpr T radian_to_degree(T radian)
{
  static_assert(std::is_floating_point<T>::value, "Floating point required.");
  return radian * 180 / pi();
}

/** Bounds the input to an interval.
 *
 * Return at most hi, at least low and otherwise the input.
 *
 * \param[in] v  The input value to clamp
 * \param[in] lo The lower bound used for clipping
 * \param[in] hi The upper bound use for clipping
 *
 * \return Returns the clampped value.
 */
template <class T, class Compare = std::less<T>>
constexpr const T &clamp(const T &v, const T &lo, const T &hi, Compare comp = Compare())
{
  return comp(v, lo) ? lo : comp(hi, v) ? hi : v;
}

/** Interpolate a 1-D function linearily.
 *
 * Computes the linear interpolation based on the given function approximation. If the given x-value
 * is outside of the defined area the corresponding edge of the function will be used.
 *
 * \param[in] x                       The interpolation location.
 * \param[in] function_approximation  Approximation of the function to interpolate. The first values
 * are x and the second function (f(x)) values.
 * \return The interpolated function value at x: f(x).
 */
template <typename T>
T interpolate(const T x, const std::vector<std::pair<T, T>> &function_approximation)
{
  static_assert(std::is_floating_point<T>::value, "Floating point required.");
  const auto x1 = std::find_if(function_approximation.cbegin(), function_approximation.cend(),
                               [&x](const std::pair<double, double> &value) { return x < value.first; });
  if (x1 == function_approximation.cbegin())
  {
    return function_approximation.front().second;
  }
  else if (x1 == function_approximation.cend())
  {
    return function_approximation.back().second;
  }
  else
  {  // value inside of function approximation
    const auto x0 = x1 - 1;
    return x0->second + (x1->second - x0->second) / (x1->first - x0->first) * (x - x0->first);
  }
}

/** Scales the value from the given range to the second one.
 *
 * \param[in] val Input value
 * \param[in] from Source range
 * \param[in] from Target range
 *
 * \return The scaled value
 */
template <typename T>
constexpr T scale_to_range(const T val, const std::pair<T, T> &from, const std::pair<T, T> &to)
{
  return (clamp(val, from.first, from.second) - from.first) / (from.second - from.first) * (to.second - to.first) +
         to.first;
}

/** Return evenly spaced numbers over a specified interval.
 *
 * \param[in] interval The interval.
 * \parm[in] n The number of elements.
 *
 * \return Evenly spaced numbers over the specified interval
 */
template <typename T = double>
std::vector<T> linspace(const std::pair<T, T> &interval, size_t n)
{
  std::vector<T> xs(n);
  if (n > 1)
  {
    T step = (interval.second - interval.first) / static_cast<T>(n - 1);
    for (size_t i = 0; i < xs.size(); i++)
    {
      xs[i] = i * step + interval.first;
    }
  }
  else
  {
    xs[0] = (interval.second - interval.first) / 2;
  }
  return xs;
}
}  // namespace cooperative_driving
