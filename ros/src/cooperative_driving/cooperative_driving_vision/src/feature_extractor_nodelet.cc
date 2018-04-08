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
#include <stdexcept>
#include <chrono>

#include "cooperative_driving/disable_ros_warnings_pre.h"
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include "cooperative_driving/disable_ros_warnings_post.h"

#include "cooperative_driving/feature_extractor.h"
#include "cooperative_driving/util.h"
#include "cooperative_driving/disable_ros_warnings_pre.h"
#include "cooperative_driving_vision/Features.h"
#include "cooperative_driving_vision/FeatureExtractorServerConfig.h"
#include "cooperative_driving_vision/Features.h"
#include "cooperative_driving/disable_ros_warnings_post.h"
#include "cooperative_driving_vision/Features.h"

using cooperative_driving_vision::Features;
using namespace std;
using namespace std::chrono;

namespace cooperative_driving
{
/** Extracts features from an image.
 *
 * The FeatureExtractionNodelet will subscribe to an image. When it
 * receives a new image it extracts features from it. These features
 * are currently:
 * - Vertical lines: The extractor will search for vertical edges
 *   which form a line. Right now, only one line is extracted.
 * - Colored regions: The extractor will look for continuous regions
 *   in the image with color that matches a configured range. The
 *   regions are described by their geometric moments.
 * The extracted features will then be published.
 *
 * Subscribed topics:
 * /camera/image_raw
 *   The source image
 *
 * Published Topics
 * features
 *   The extracted features
 *
 * Parameters
 * color_ranges
 *   Ranges of colors within which the extracted Regions will be.
 *   Represented as a dictionary indexed by the color name and both a
 *   'lower_bound' and 'upper_bound' RGB-tuple.
 * min_region_size (int)
 *   The minimum amount of pixels valid Regions have to contain.
 * normalized_row_location
 *   Normalized y-location for line detection.
 * row_influence_area
 *   Number of rows to include for line detection.
 */
class FeatureExtractionNodelet : public nodelet::Nodelet
{
public:
  /** Initializes static data.
   */
  FeatureExtractionNodelet() : line_interval_v(0, 1)
  {
  }

  /** Initializes the ROS interface, i.e. publishers, subscriptions and parameters.
   */
  void onInit() override
  {
    auto nh = getNodeHandle();
    auto private_nh = getPrivateNodeHandle();
    auto it = image_transport::ImageTransport(nh);

    std::vector<double> lower_color_bound, upper_color_bound;
    // Sum a list of doubles from the parameter server
    XmlRpc::XmlRpcValue color_list;
    private_nh.getParam("color_ranges", color_list);
    ROS_ASSERT(color_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = color_list.begin(); it != color_list.end(); ++it)
    {
      std::string color_name = (std::string)(it->first);
      std::string lower = std::string("color_ranges/") + color_name + std::string("/lower_bound");
      std::string upper = std::string("color_ranges/") + color_name + std::string("/upper_bound");
      if (private_nh.getParam(lower, lower_color_bound) && lower_color_bound.size() == 3 &&
          private_nh.getParam(upper, upper_color_bound) && upper_color_bound.size() == 3)
      {
        auto vec_pair = std::make_pair(cv::Vec3d(lower_color_bound.data()), cv::Vec3d(upper_color_bound.data()));
        color_ranges_.insert(std::pair<std::string, ColorRange>(color_name, vec_pair));
        ROS_INFO_STREAM("Color: " << color_name << " found and set.");
      }
    }
    ROS_ASSERT(color_ranges_.size() > 0);

    reconfigure_server_.reset(new ReconfigureServer(private_nh));
    reconfigure_server_->setCallback(boost::bind(&FeatureExtractionNodelet::parameterServerCallback, this, _1, _2));

    feature_publisher_ = private_nh.advertise<Features>("features", 1);
    image_subscriber_ = it.subscribeCamera("/camera/image_raw", 1, &FeatureExtractionNodelet::onCameraImage, this);
  }

  /** Callback for received image.
   *
   * Triggers feature-extraction and assembles a new Features-message for the result.
   *
   * \param image_msg The image message
   */
  void onCameraImage(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr & /*info_msg*/)
  {
    // load camera calibration file
    /*if (camera_model_.fromCameraInfo(info_msg)) {
      if (!transform_listener_.canTransform(camera_model_.tfFrame(), "base_link", ros::Time())) {
      NODELET_ERROR_STREAM("Can't transform camera coordinates, shutting down.");
      getNodeHandle().shutdown();
      return;
      }
    }*/

    high_resolution_clock::time_point tmp_timestamp = last_picture_process_start;
    last_picture_process_start = high_resolution_clock::now();

    auto time_since_last_call = duration_cast<milliseconds>( last_picture_process_start - tmp_timestamp ).count();
    float pfps = 1000/time_since_last_call;

    // prepare received image for further processing
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    int width = cv_ptr->image.cols; //box_width * cv_ptr->image.cols;
    int height = box_height * cv_ptr->image.rows;

    int x_box_offset = (cv_ptr->image.cols - width) / (static_cast<double>(2));
    int y_box_offset = cv_ptr->image.rows - height;

    cv::Mat cropped_image = cv_ptr->image(cv::Rect(0 , y_box_offset, width, height));   // black

    cv::Mat reducedImage;
    reduce(cropped_image, reducedImage, 0, CV_REDUCE_AVG);
    reduce(reducedImage, reducedImage, 1, CV_REDUCE_AVG);

    cv::Mat imageDest = cv::Mat(cropped_image.rows, cropped_image.cols, CV_8UC3); // source
    imageDest = imageDest.setTo(cv::Scalar(reducedImage.at<cv::Vec3b>(cv::Point(0,0))[0],reducedImage.at<cv::Vec3b>(cv::Point(0,0))[1],reducedImage.at<cv::Vec3b>(cv::Point(0,0))[2]));
    
    cv::Mat mask(imageDest.rows, imageDest.cols, CV_8UC1, cv::Scalar(0)); // mask

    std::vector<std::vector<cv::Point>> coordinates;
    coordinates.push_back(std::vector<cv::Point>());
    coordinates[0].push_back(cv::Point(0, imageDest.rows));
    coordinates[0].push_back(cv::Point(imageDest.cols/(static_cast<double>(2)) * box_height, 0));
    coordinates[0].push_back(cv::Point(imageDest.cols - imageDest.cols/(static_cast<double>(2)) * box_height, 0));
    coordinates[0].push_back(cv::Point(imageDest.cols, imageDest.rows));
    cv::drawContours(mask, coordinates, 0, cv::Scalar(255), CV_FILLED, 8);

    cropped_image.copyTo(imageDest, mask);

    Features feature_msg;
    feature_msg.header.stamp = image_msg->header.stamp;
    feature_msg.image_width = cv_ptr->image.cols;
    feature_msg.image_height = cv_ptr->image.rows;
    feature_msg.PFPS = pfps;
    feature_msg.box_width = width;
    feature_msg.box_height = height;

    high_resolution_clock::time_point extract_lines_begin = high_resolution_clock::now();
    for (const auto &normalized_line_row : linspace(line_interval_v, line_count_v))
    {
      int line = extract_vertical_line(imageDest, normalized_line_row, row_influence_area_v, lower_canny_threshold_v);

      if (line >= 0)
      {
        geometry_msgs::Point line_msg;
        line_msg.x = line + x_box_offset;
        line_msg.y = (1 - normalized_line_row) * imageDest.rows + y_box_offset;
        feature_msg.Vlines.push_back(line_msg);
      }
    }

    for (const auto &normalized_line_row : linspace(line_interval_h, line_count_h))
    {
      int line = extract_horizontal_line(imageDest, normalized_line_row, row_influence_area_h, lower_canny_threshold_h);

      if (line >= 0)
      {
        geometry_msgs::Point line_msg;
        line_msg.y = line + y_box_offset;
        line_msg.x = (1 - normalized_line_row) * imageDest.cols + x_box_offset;
        feature_msg.Hlines.push_back(line_msg);
      }
    }
    high_resolution_clock::time_point extract_lines_end = high_resolution_clock::now();

    auto lines_extraction_duration = duration_cast<milliseconds>( extract_lines_end - extract_lines_begin ).count();

    feature_publisher_.publish(feature_msg);
  }

private:
  typedef cooperative_driving_vision::FeatureExtractorServerConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  high_resolution_clock::time_point last_picture_process_start;

  /** Dynamic-reconfiguration callback
   *
   * Updates the dynamicaly configurable parameters.
   *
   * \param config New values
   */
  void parameterServerCallback(const Config &config, uint32_t /*level*/)
  {
    if (config.line_interval_lower_v > config.line_interval_upper_v)
    {
      std::runtime_error("Invalid vertical interval bounds (lower > upper)");
    }
    line_interval_v.first = config.line_interval_lower_v;
    line_interval_v.second = config.line_interval_upper_v;
    line_count_v = config.line_count_v;
    row_influence_area_v = config.row_influence_area_v;
    lower_canny_threshold_v = config.lower_canny_threshold_v;

    if (config.line_interval_lower_h > config.line_interval_upper_h)
    {
      std::runtime_error("Invalid horizontal interval bounds (lower > upper)");
    }
    line_interval_h.first = config.line_interval_lower_h;
    line_interval_h.second = config.line_interval_upper_h;
    line_count_h = config.line_count_h;
    row_influence_area_h = config.row_influence_area_h;
    lower_canny_threshold_h = config.lower_canny_threshold_h;

    box_width = config.box_width;
    box_height = config.box_height;

    min_line_length_ = config.min_line_length;
    min_region_size_ = config.min_region_size;
  }

  ros::Publisher feature_publisher_; //!< Publishes Features messages containing the extracted features.
  image_transport::CameraSubscriber image_subscriber_; //!< Subscription for the source image.
  std::map<std::string, ColorRange> color_ranges_;     //!< ColorRanges for regions of interest.
  int min_region_size_;                                //!< The minimun size detected regions are processed.
  int min_line_length_;                                //!< The minimum line length for detecting blobs.
  tf::TransformListener transform_listener_;           //!< Listener to transmformations for transformaing camera- into world-coordinates. Currently unused.
  image_geometry::PinholeCameraModel camera_model_;    //!< Camera information for un-projecting image-coordinates. Currently unused.
  std::unique_ptr<ReconfigureServer> reconfigure_server_; //!<< Dynamic configuration service.

  std::pair<float, float> line_interval_v;                 //!<< Range of normalized rows to lookup lines in the image.
  int line_count_v;                                        //!<< Number of lines to extract.
  int row_influence_area_v;                                //!<< Number of rows to include for line detection.
  int lower_canny_threshold_v; //!<< Threshold used for the canny filter to detect lines. Max threshold is 3 times the lower threshold.

  std::pair<float, float> line_interval_h;                 //!<< Range of normalized rows to lookup lines in the image.
  int line_count_h;                                        //!<< Number of lines to extract.
  int row_influence_area_h;                                //!<< Number of rows to include for line detection.
  int lower_canny_threshold_h; //!<< Threshold used for the canny filter to detect lines. Max threshold is 3 times the lower threshold.

  double box_width;
  double box_height;
};

}  // namespace cooperative_driving

#include "cooperative_driving/disable_ros_warnings_pre.h"
PLUGINLIB_EXPORT_CLASS(cooperative_driving::FeatureExtractionNodelet, nodelet::Nodelet)
#include "cooperative_driving/disable_ros_warnings_post.h"
