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
#include <controller_interface/controller.h>
#include "cooperative_driving/disable_ros_warnings_post.h"
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>

#include "cooperative_driving/distance_sensor_interface.h"
#include "cooperative_driving/disable_ros_warnings_pre.h"
#include "cooperative_driving_hardware/DistanceValues.h"
#include "cooperative_driving/disable_ros_warnings_post.h"

namespace cooperative_driving
{
/** Controller for accessing sensors via the DistanceSensorInterface
 *
 * The class DistanceSensorController stores a DistanceSensorHandle
 * for every registered sensor to publishs its readings with a RealtimePublisher in
 * a DistanceValues message.
 */
class DistanceSensorController : public controller_interface::Controller<DistanceSensorInterface>
{
public:
  /** Constructor of DistanceSensorInterface class */
  DistanceSensorController() : publish_rate_(0)
  {
  }

  /** Initiliazes the controller
   *
   * Reads parameters from ROS and initializes members.
   *
   * \param[in] hw The interface to access the sensors
   * \param[in] root_nh The handle for the root node
   * \param[in] controller_nh The handle for the controller node
   */
  bool init(DistanceSensorInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  /** Starts the controller
   *
   * \param[in] time The curent time
   */
  void starting(const ros::Time& time) override;

  /** Updates the state of the managed sensors
   *
   * This method is called periodically by ROS to read the state of the sensors.
   * It publishes a message containing the state of all sensors on topic 'distances'.
   *
   * \param[in] time The current time
   * \param[in] period Unused
   */
  void update(const ros::Time& time, const ros::Duration& /*period*/) override;

  /** Stops the controller
   *
   * \param[in] time Unused
   */
  void stopping(const ros::Time& /*time*/) override
  {
  }

private:
  typedef boost::shared_ptr<realtime_tools::RealtimePublisher<cooperative_driving_hardware::DistanceValues>>
      RtPublisherPtr;  ///< Publisher type

  std::string name_;                           ///< The name of the controller
  std::vector<DistanceSensorHandle> sensors_;  ///< Handles of the sensors
  RtPublisherPtr realtime_pub_;  ///< A publisher for cooperative_driving_hardware::DistanceValues messages on the
                                 ///'distances' topic
  ros::Time last_publish_time_;  ///< The last time a cooperative_driving_hardware::DistanceValues message was published
  ros::Rate publish_rate_;  ///< The rate at which a cooperative_driving_hardware::DistanceValues mesages is published
};

}  // namespace cooperative_driving

#include "cooperative_driving/disable_ros_warnings_pre.h"
PLUGINLIB_EXPORT_CLASS(cooperative_driving::DistanceSensorController, controller_interface::ControllerBase)
#include "cooperative_driving/disable_ros_warnings_post.h"
