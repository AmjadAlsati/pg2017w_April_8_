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
#include <math.h>
#include <deque>
#include <functional>

#include "cooperative_driving/disable_ros_warnings_pre.h"
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/Config.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include "cooperative_driving/disable_ros_warnings_post.h"

#include "cooperative_driving/region.h"
#include "cooperative_driving/vision_util.h"
#include "cooperative_driving/disable_ros_warnings_pre.h"
#include "cooperative_driving_vision/Features.h"
#include "cooperative_driving/disable_ros_warnings_post.h"

namespace cooperative_driving
{

/**
   * Calculates angle between two given points.
   *
   * \param[in] v1 First Point
   * \param[in] v2 Second Point
   */
float angleBetween(const cv::Point2f &v1, const cv::Point2f &v2)
  {
      float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
      float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

      float dot = v1.x * v2.x + v1.y * v2.y;

      float a = dot / (len1 * len2);

      if (a >= 1.0)
          return 0.0;
      else if (a <= -1.0)
          return M_PI;
      else
          return acos(a); // 0..PI
  }
  
  /** Check if there is an intersection between two lines.
   *
   * Finds the intersection of two lines, or returns false.
   * The lines are defined by (o1, p1) and (o2, p2).
   *
   * \param[in] o1 First Point
   * \param[in] p1 Second Point
   * \param[in] o2 Third Point
   * \param[in] p2 Fourth Point
   * \param[inout] r Location of the crossing, if there is any
   */
  bool containsIntersections(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f &r)
  {
      cv::Point2f x = o2 - o1;
      cv::Point2f d1 = p1 - o1;
      cv::Point2f d2 = p2 - o2;

      float angle = angleBetween(d1, d2);

      double treshold_lower = M_PI / (static_cast<double>(2)) - 0.25;
      double treshold_upper = M_PI / (static_cast<double>(2)) + 0.25;

      if (angle <=  treshold_lower || angle >= treshold_upper) {
        return false;
      }

      float cross = d1.x*d2.y - d1.y*d2.x;
      if (std::abs(cross) < /*EPS*/1e-16)
          return false;

      double t1 = (x.x * d2.y - x.y * d2.x)/cross;
      r = o1 + d1 * t1;
      return true;
  }

/** Renders the Features on top of the given image.
 *
 * \param[inout] image The target image.
 * \param features The features.
 */
static void render_features(cv::Mat &image, const cooperative_driving_vision::FeaturesConstPtr &features,
                            const double logic_line_lookup_location)
{
  std::vector<std::vector<cv::Point>> coordinates;
  coordinates.push_back(std::vector<cv::Point>());
  coordinates[0].push_back(cv::Point(0, image.rows));
  coordinates[0].push_back(cv::Point(image.cols/(static_cast<double>(2)) * (features->box_height/static_cast<double>(image.rows)), image.rows-features->box_height));
  coordinates[0].push_back(cv::Point(image.cols - image.cols/(static_cast<double>(2)) * (features->box_height/static_cast<double>(image.rows)), image.rows-features->box_height));
  coordinates[0].push_back(cv::Point(image.cols, image.rows));
  
  cv::drawContours(image, coordinates, 0, CV_RGB(255, 255, 0), 2);
  
  if (features->Vlines.size() > 1 && features->Hlines.size() > 1) {
    cv::Point2f p1_v = cv::Point2f(features->Vlines.at(0).x, features->Vlines.at(0).y);
    cv::Point2f p2_v = cv::Point2f(features->Vlines.at(features->Vlines.size()-1).x, features->Vlines.at(features->Vlines.size()-1).y);
    cv::Point2f p1_h = cv::Point2f(features->Hlines.at(0).x, features->Hlines.at(0).y);
    cv::Point2f p2_h = cv::Point2f(features->Hlines.at(features->Hlines.size()-1).x, features->Hlines.at(features->Hlines.size()-1).y);

    cv::Point2f crossing;
    bool contains_intersection = containsIntersections(p1_v, p2_v, p1_h, p2_h, crossing);

    if (contains_intersection) {
      cv::circle(image, { crossing.x, crossing.y }, 5, CV_RGB(255, 255, 255), 3);
    }
  }

  // draw vertical lines
  for (auto it = std::make_pair(features->Vlines.begin(), std::next(features->Vlines.begin()));
       it.first != features->Vlines.end(); it.first++, it.second++)
  {
    cv::circle(image, { (int)it.first->x, (int)it.first->y }, 5, CV_RGB(0, 0, 255), 1);
    if (it.second != features->Vlines.end())
    {
      cv::line(image, { (int)it.first->x, (int)it.first->y }, { (int)it.second->x, (int)it.second->y }, CV_RGB(0, 0, 255),
               1);
    }
  }

  // draw horizontal lines
  for (auto it = std::make_pair(features->Hlines.begin(), std::next(features->Hlines.begin()));
       it.first != features->Hlines.end(); it.first++, it.second++)
  {
    cv::circle(image, { (int)it.first->x, (int)it.first->y }, 5, CV_RGB(255, 0, 0), 1);
    if (it.second != features->Hlines.end())
    {
      cv::line(image, { (int)it.first->x, (int)it.first->y }, { (int)it.second->x, (int)it.second->y }, CV_RGB(255, 0, 0),
               1);
    }
  }

  cv::putText(image, "PFPS: " + std::to_string(features->PFPS), {10, 30}, CV_FONT_HERSHEY_PLAIN, 2, CV_RGB(255, 0, 0), 3);

  if (logic_line_lookup_location >= 0 && features->Vlines.size() > 0)
  {
    int line_height = image.cols / 32;
    int line_lookup_row = (1 - logic_line_lookup_location) * image.rows;
    int line_location_x = static_cast<int>(line_location(line_lookup_row, features->Vlines));
    cv::circle(image, { line_location_x, line_lookup_row }, 5, { 0, 0, 255 }, 2);
    std::stringstream s;
    s << "x=" + std::to_string(line_location_x) << ", y=" << std::to_string(line_lookup_row);
    cv::putText(image, s.str(), { line_location_x + 5, line_lookup_row - line_height / 2 }, 0, 0.5, { 0, 0, 255 }, 1,
                CV_AA);
  }

  // draw regions
  for (const auto region_msg : features->regions)
  {
    const Region region = region_msg;
    region.draw(image, true);

    std::stringstream text;
    text << std::fixed << std::setprecision(0) << "(" << region.moment.center().x << "," << region.moment.center().y
         << ")";
    cv::putText(image, text.str(), region.moment.center(), 0, 0.3, { 255, 255, 255 }, 1, CV_AA);
  }
}

/** Publishes an image showing the detected features in an image.
 *
 * The original image and features are subscribed to and composed
 * into a single image. Only messages with the same timestamp will be associated
 * with each other.
 *
 * \todo The topics should be more generalized, less reliant on the actual launch
 * file.
 *
 * Subscribed topics:
 * /feature_extractor/features(cooperative_driving_vision/Features)
 *   The Features that should be rendered.
 * /camera/image_raw(sensor_msgs/Image)
 *   The original image, i.e. the one the features where extracted from.
 *
 * Published topics:
 * /feature_extractor/feature_image(sensor_msgs/Image)
 *   The combined image.
 *
 */
class FeatureImageRenderer
{
public:
  /** Initializes the FeatureImageRenderer based on the given NodeHandles.
   *
   * \param node_handle: The root NodeHandle.
   * \param private_node_handle: The local NodeHandle.
   */
  FeatureImageRenderer(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : image_transport_(private_node_handle)
    , feature_subscriber_(
          node_handle.subscribe("/feature_extractor/features", 1, &FeatureImageRenderer::features_callback, this))
    , image_subscriber_(image_transport_.subscribe("/camera/image_raw", 1, &FeatureImageRenderer::image_callback, this,
                                                   image_transport::TransportHints("compressed")))
    , logic_parameter_subscriber_(node_handle.subscribe("/reflekte/parameter_updates", 1,
                                                        &FeatureImageRenderer::logic_parameter_update_callback, this))
    , image_publisher_(image_transport_.advertise("/feature_extractor/feature_image", 1))
    , feature_messages_()
    , image_messages_()
    , logic_line_lookup_location_(-1)
  {
  }

private:
  /** Pushes the features into the queue and publishes all new combined images.
   *
   * \see publish_images()
   */
  void features_callback(const cooperative_driving_vision::FeaturesConstPtr &feature_message)
  {
    feature_messages_.push_back(feature_message);
    publish_images();
  }

  /** Pushes the image into the queue and publishes all new combined images.
   *
   * \see publish_images()
   */
  void image_callback(const sensor_msgs::ImageConstPtr &image_message)
  {
    image_messages_.push_back(image_message);
    publish_images();
  }

  /** Sets the lookup location used by the logic's line-following.
   */
  void logic_parameter_update_callback(const dynamic_reconfigure::Config &config)
  {
    for (const auto &double_parameter : config.doubles)
    {
      if (double_parameter.name == "normalized_line_row")
      {
        logic_line_lookup_location_ = double_parameter.value;
      }
    }
  }

  /** Finds matching pairs in image_messages_ and feature_messages_, renders the corresponding images and publishes them.
   */
  void publish_images()
  {
    auto image_message_it = image_messages_.begin();
    auto feature_message_it = feature_messages_.begin();

    while (image_message_it != image_messages_.end() && feature_message_it != feature_messages_.end())
    {
      const auto &image_timestamp = (*image_message_it)->header.stamp;
      const auto &features_timestamp = (*feature_message_it)->header.stamp;
      if (image_timestamp == features_timestamp)
      {
        auto image = cv_bridge::toCvCopy(*image_message_it);
        render_features(image->image, *feature_message_it, logic_line_lookup_location_);
        image_publisher_.publish(image->toImageMsg());

        image_messages_.erase(image_messages_.begin(), image_message_it);
        feature_messages_.erase(feature_messages_.begin(), feature_message_it);

        std::advance(image_message_it, 1);
        std::advance(feature_message_it, 1);
      }
      else if (image_timestamp < features_timestamp)
      {
        std::advance(image_message_it, 1);
      }
      else if (image_timestamp > features_timestamp)
      {
        std::advance(feature_message_it, 1);
      }
    }
  }

  image_transport::ImageTransport image_transport_; //!< The image transport handle.
  ros::Subscriber feature_subscriber_;              //!< The feature subscriber.
  image_transport::Subscriber image_subscriber_;    //!< The image subscriber.
  ros::Subscriber logic_parameter_subscriber_;      //!< Subscriber to logic parameter updates.
  image_transport::Publisher image_publisher_;      //!< The publisher for the composed image.
  std::deque<cooperative_driving_vision::FeaturesConstPtr> feature_messages_; //!< List of unhandled Feature messages.
  std::deque<sensor_msgs::ImageConstPtr> image_messages_;                     //!< List of unhandled Image messages.
  double logic_line_lookup_location_; //!< The normalized row which the logic node uses for line following.
};

}  // namespace cooperative_driving

int main(int argc, char *argv[])
{
  std::string name = ros::getROSArg(argc, argv, "__name");
  if (name == "")
    name = "feature_image_renderer";

  ros::init(argc, argv, name);
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");
  cooperative_driving::FeatureImageRenderer feature_image_renderer(node_handle, private_node_handle);
  ROS_INFO_STREAM("Initialized node " << name);

  ros::spin();

  return 0;
}
