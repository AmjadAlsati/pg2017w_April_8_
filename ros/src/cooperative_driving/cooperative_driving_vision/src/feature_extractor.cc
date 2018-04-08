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

/** 
 * TODO:
 *  - Clean up namespace usages
 */

#include "cooperative_driving/feature_extractor.h"

#include <memory>
#include <numeric>

#include <ros/console.h>

#include "cooperative_driving/moment.h"
#include "cooperative_driving/vision_util.h"

using cv::Mat;
using cv::Mat_;
using cv::Mat3b;
using cv::Point;
using cv::Size;
using std::vector;
using std::shared_ptr;

namespace cooperative_driving
{
/** Structure used for efficiently extracting colored regions from an image.
 *
 * The algorithm first detects regions in single lines that are matching the specified color regions.
 * In a second step, these regions are merged into two dimensional regions.
 * For bookkeeping two matrices (\see {line_region_locations merged_region_locations}) are used to associate
 * locations with the corresponding LineRegion/MergedRegion. Since updating these when merging two
 * MergedRegions is too expensive, each MergedRegion contains a reference to the its parent-MergedRegion.
 * Only MergedRegions with a null-parent-reference are valid top-level regions and include all relevant
 * pixels.
 */
class EREX
{
public:
  /** Creates a region extractor for images of the given size and the given ColorRanges.
   *
   * \param size The image size.
   * \param color_ranges color-ranges of valid regions.
   */
  EREX(const Size &size, const vector<ColorRange> &color_ranges)
    : color_ranges_(color_ranges), region_descriptors_(size.height), regions_()
  {
  }

  /** Extracts Regions from the given image.
    *
    * \todo Test parallelization.
    *
    * \param[in] frame The reference image on which the extractor operates
    * \param[in] min_size  The minimum size of blobs that should be considered
    * \param[in] min_line_lenght The minimum size at which line segments should be considered
    *
    * \return A vector of Regions that has been extracted
    */
  vector<Region> operator()(const Mat &frame, const unsigned min_size, const int min_line_length)
  {
    for (int row = 0; row < frame.rows; row++)
    {
      region_descriptors_[row] = extract_line_regions(frame, row);
      region_descriptors_[row].erase(std::remove_if(region_descriptors_[row].begin(), region_descriptors_[row].end(),
                                                    [min_line_length](const RegionDescriptor &desc) {
                                                      return desc.end - desc.start < min_line_length;
                                                    }),
                                     region_descriptors_[row].end());
    }

    for (int row = 1; row < frame.rows; row++)
    {
      extract_merged_regions(row);
    }

    // render_intermediate_regions(const_cast<Mat&>(frame));

    std::sort(regions_.begin(), regions_.end());
    regions_.erase(std::unique(regions_.begin(), regions_.end()), regions_.end());
    vector<Region> result;
    for (auto &region_ptr : regions_)
    {
      if (region_ptr->moment.m00() >= min_size)
      {
        result.push_back(*region_ptr);
      }
    }
    return result;
  }

  /** Render extract_features's intermediate results to the image.
  *
  * Use only for debugging purposes.
  *
  * \param[in] img The image on which intermediate result should be drawn.
  */
  void render_intermediate_regions(Mat &img)
  {
    const vector<Color> line_color_map = { Color(255, 0, 0), Color(0, 255, 0), Color(0, 0, 255) };
    for (const auto &regions : region_descriptors_)
    {
      for (auto it = regions.begin(); it != regions.end(); it++)
      {
        Color color =
            it->line() ? line_color_map[(it - regions.begin()) % 3] : regions_[it->region_idx]->average_color();
        line(img, { it->start, it->row }, { it->end - 1, it->row }, color);
      }
    }
  }

private:
  /** Describes either a line- or a merged-region.
   *
   * A type-flag identifies the type. While a line-RegionDescriptor is self-contained,
   * i.e. all relevant information (except the row) are present in the struct, a
   * merged-RegionDescriptor refrences a Region which contains the moments as well
   * as the average color.
   */
  struct RegionDescriptor
  {
    /** The possible descriptor types. */
    enum Type
    {
      LINE,
      MERGED
    };

    /** Creates a new RegionDescriptor with the given attributes. */
    RegionDescriptor(const int row, const int start, Color color, const ColorRange &color_range)
      : row(row), start(start), end(start + 1), color_range(color_range), type(LINE), color_sum(color)
    {
    }

    /** Adds a pixel with the given color to the region.
     *
     * The pixel must have the location (row_index, column_index + length).
     *
     * \param[in] color The color that should be added
     */
    inline void add(const Color &color)
    {
      assert(line() && checkInRange(color, color_range));
      color_sum += color;
      end += 1;
    }

    /** line-RegionDescriptor predicate. */
    inline bool line() const
    {
      return type == LINE;
    }

    /** Merged-RegionDescriptor predicate. */
    inline bool merged() const
    {
      return type == MERGED;
    }

    /** Converts this descriptor to a merged-descriptor.
     *
     * This will add the contents to the color_sum and moment of the given Region.
     *
     * \param region_idx The parent Region index in regions_.
     * \param row The row of this RegionDescriptor.
     */
    void to_merged(vector<shared_ptr<Region>> &regions, const int region_idx)
    {
      assert(line());
      regions[region_idx]->color_sum += color_sum;
      const auto length = end - start;
      const uint64_t column_sum = gauss_sum(end) - gauss_sum(start);
      const uint64_t squared_column_sum = square_pyramidal_number(end) - square_pyramidal_number(start);
      const uint64_t cubed_column_sum = column_sum * column_sum;
      regions[region_idx]->moment +=
          Moment(
              length,
              column_sum,
              row * length,
              row * column_sum,
              squared_column_sum,
              row * row * length,
              column_sum * row,
              row * std::pow(row, 2),
              cubed_column_sum,
              std::pow(row, 3) * length);

      type = MERGED;
      this->region_idx = region_idx;
    }

    int row;                 //!< The region's row.
    int start;               //!< The starting column.
    int end;                 //!< The ending column.
    ColorRange color_range;  //!< Boundary for color's of all included pixels.
    Type type;               //!< The region's type.
    ColorSum color_sum;      //!< Sum of all included pixels' colors. Valid only for line-RegionDescriptors.
    int region_idx;          //!< The Region this descriptor refers to. Valid only for merged-RegionDescriptors.
  };

/** Detects contiguous regions of pixels matching one of the specified color_ranges_ in a row.
 *
 * \param[in] frame Reference to the image
 * \param[in] row The row index
 *
 * \return A vector of RegionDescriptor representing the extracted lines
 */
  vector<RegionDescriptor> extract_line_regions(const Mat &frame, const int row)
  {
    RegionDescriptor *current_region = nullptr;
    vector<RegionDescriptor> line_regions;

    for (int col = 0; col < frame.cols; col++)
    {
      const auto current_color = frame.at<Color>(row, col);
      const auto current_range = findMatchingRange(current_color, color_ranges_);

      const bool current_range_valid = current_range != color_ranges_.end();
      const auto region_valid = [](RegionDescriptor *region) { return region != nullptr; };
      const bool matching_color_range =
          region_valid(current_region) && current_range_valid && current_region->color_range == *current_range;
      const bool non_matching_color_range =
          region_valid(current_region) &&
          (!current_range_valid || (current_range_valid && current_region->color_range != *current_range));

      if (matching_color_range)
      {
        current_region->add(current_color);
      }
      else
      {
        if (non_matching_color_range)
        {
          current_region = nullptr;
        }
        if (!region_valid(current_region) && current_range_valid)
        {
          line_regions.push_back(RegionDescriptor(row, col, current_color, *current_range));
          current_region = &(line_regions.back());
        }
      }
    }
    return line_regions;
  }

  /** Create and extend MergedRegions by comparing regions of this line to those of the previous one.
   *
   * Depending on the regions that are found at the sampled locations, this will
   * result in different actions:
   * - Create a new merged region (two line regions found)
   * - Merge a line region into merged regions
   * - Merge two merged regions
   *
   * \see generate_sample_points
   * \see new_region
   *
   * \param[in] row The row that is currently processed
   */
  void extract_merged_regions(const int row)
  {
    auto &current_line = region_descriptors_[row];
    auto &previous_line = region_descriptors_[row - 1];
    auto current_region = current_line.begin();
    auto previous_region = previous_line.begin();

    while (current_region != current_line.end() && previous_region != previous_line.end())
    {
      if (regions_mergeable(*current_region, *previous_region))
      {
        if (current_region->line() && previous_region->line())
        {
          new_region(*current_region, *previous_region);
        }
        else if (current_region->line() && previous_region->merged())
        {
          merge_into(*previous_region, *current_region);
        }
        else if (current_region->merged() && previous_region->line())
        {
          merge_into(*current_region, *previous_region);
        }
        else
        {  // two merged regions
          combine_merged_regions(*current_region, *previous_region);
        }
      }
      if (current_region->end < previous_region->end)
      {
        current_region += 1;
      }
      else
      {
        previous_region += 1;
      }
    }
  }

  /** Checks whether the given regions are mergeable.
   *
   * The check confirms that the region's locations intersect and their color-ranges match.
   *
   * \param lhs The first region.
   * \param rhs The second region
   *
   * \return True if the two regions could be merged
   */
  inline bool regions_mergeable(const RegionDescriptor &lhs, const RegionDescriptor &rhs)
  {
    return lhs.start <= rhs.end && lhs.end > rhs.start && lhs.color_range == rhs.color_range;
  }

  /** Combine two line regions into a new merged region.
   *
   * Updates the list of regions accordingly.
   *
   * \param[in] lhs The left hand side for the comparison
   * \param[in] rhs The right hand side for the comparison
   */
  void new_region(RegionDescriptor &lhs, RegionDescriptor &rhs)
  {
    assert(lhs.line() && rhs.line());
    regions_.push_back(std::make_shared<Region>());
    lhs.to_merged(regions_, regions_.size() - 1);
    rhs.to_merged(regions_, regions_.size() - 1);
  }

  /** Merge a line region into a merged region.
   *
   * \param merged_region The target merged region.
   * \param line_region The line region that will be merged.
   */
  void merge_into(const RegionDescriptor &merged_region, RegionDescriptor &line_region)
  {
    assert(merged_region.merged() && line_region.line());
    line_region.to_merged(regions_, merged_region.region_idx);
  }

  /** Combine two merged Regions.
   *
   * This will extend the first Region by the second one.
   *
   * \param[in] lhs The left hand side for the comparison
   * \param[in] rhs The right hand side for the comparison
   */
  void combine_merged_regions(const RegionDescriptor &lhs, const RegionDescriptor &rhs)
  {
    assert(lhs.merged() && rhs.merged());
    auto &lhs_ptr = regions_[lhs.region_idx];
    const auto rhs_ptr = regions_[rhs.region_idx];
    if (lhs_ptr != rhs_ptr)
    {  // check for equality on second indirection level
      *lhs_ptr += *rhs_ptr;
      std::replace(regions_.begin(), regions_.end(), rhs_ptr, lhs_ptr);
    }
  }

  const vector<ColorRange> color_ranges_;               //!< The ColorRanges of valid regions.
  vector<vector<RegionDescriptor>> region_descriptors_; //!< List of RegionDescriptor's present for each row of the image. 
                                                        //!< The sublists are ordered by their x-coordinate.
  vector<shared_ptr<Region>> regions_;                  //!< The list of detected regions.
};


/** Extracts regions from the image
 *
 * Extracts regions from the image.
 *
 * \param[in]  color_ranges     The color ranges
 * \param[in]  frame            The frame
 * \param[in]  min_line_length  The minimum line length
 * \param[in]  min_size         The minimum size
 *
 * \return  A vector of regions.
 */
std::vector<Region> extract_regions(const std::vector<ColorRange> &color_ranges, const cv::Mat &frame,
                                    const int min_line_length, const unsigned min_size)
{
  // check if frame is continuous
  if (!frame.isContinuous())
  {
    throw std::invalid_argument("Frame not continuous!");
  }
  if (frame.depth() != CV_8U)
  {
    throw std::invalid_argument("Frame not cv_8u!");
  }

  EREX extractor(frame.size(), color_ranges);
  return extractor(frame, min_size, min_line_length);
}

/** Extracts vertical lines from given image.
 * 
 * Part of the image is reduced to one row and canny edge detection is applied to the single row.
 *
 * \param[in]  image            The input image
 * \param[in]  height           The height
 * \param[in]  neighborhood     The neighborhood
 * \param[in]  canny_threshold  The canny threshold
 *
 * \return  x-coordinate of the detected line
 */
int extract_vertical_line(const cv::Mat &image, const float height, const int neighborhood, const int canny_threshold)
{
  using namespace cv;

  cv::Mat gray_rows, canny_output;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  const int start_row = (1 - height) * (image.rows - neighborhood);

  cvtColor(image.rowRange(start_row, start_row + neighborhood), gray_rows, cv::COLOR_BGR2GRAY);
  reduce(gray_rows, gray_rows, 0, CV_REDUCE_AVG);
  blur(gray_rows, gray_rows, cv::Size(3, 3));
  Canny(gray_rows, canny_output, canny_threshold, canny_threshold * 3, 3);
  findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  if (contours.size() == 2)
  {
    return (contours.at(0).at(0).x + contours.at(1).at(0).x) / 2.0f;
  }
  else
  {
    return -1;
  }
}

/** Extracts Horizontal lines from given image.
 * 
 * Part of the image is reduced to one row and canny edge detection is applied to the single column.
 *
 * \param[in]  image            The input image
 * \param[in]  height           The height
 * \param[in]  neighborhood     The neighborhood
 * \param[in]  canny_threshold  The canny threshold
 *
 * \return  y-coordinate of the detected line
 */
int extract_horizontal_line(const cv::Mat &image, const float height, const int neighborhood, const int canny_threshold)
{
  using namespace cv;

  cv::Mat gray_cols, canny_output;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  const int start_col = (1 - height) * (image.cols - neighborhood);

  cvtColor(image.colRange(start_col, start_col + neighborhood), gray_cols, cv::COLOR_BGR2GRAY);
  reduce(gray_cols, gray_cols, 1, CV_REDUCE_AVG);
  blur(gray_cols, gray_cols, cv::Size(3, 3));
  Canny(gray_cols, canny_output, canny_threshold, canny_threshold * 3, 3);
  findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  if (contours.size() == 2)
  {
    return (contours.at(0).at(0).y + contours.at(1).at(0).y) / 2.0f;
  }
  else
  {
    return -1;
  }
}

}  // end namespace cooperative_driving
