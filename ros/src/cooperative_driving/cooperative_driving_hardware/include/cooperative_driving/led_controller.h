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
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>

#include "cooperative_driving/led_interface.h"
#include "cooperative_driving/disable_ros_warnings_pre.h"
#include "cooperative_driving_hardware/LedCommand.h"
#include "cooperative_driving_hardware/LedValues.h"
#include "cooperative_driving/disable_ros_warnings_post.h"

namespace cooperative_driving
{
/** Controller for managening LEDs via the LedInterface
 *
 * The class LedController stores a LedHandle for every registered LED
 * to access its state.
 * It publishes its value with a RealtimePublisher in a LedValues message and sets its
 * value by subscribing to LedCommand messages.
 */
class LedController : public controller_interface::Controller<LedInterface>
{
public:
  /** Constructor of LedController class */
  LedController() : publish_rate_(0), base_frame_id_("base_link"), cmd_timeout_(0)
  {
  }

  /** Destructor of LedController class
   *
   * Switches off the LEDs.
   */
  ~LedController()
  {
    switch_off();
  }

  /** Initilaizes the controller
   *
   * Reads parameters from ROS and intializes members.
   *
   * \param[in] hw The interface to access the LEDs
   * \param[in] root_nh The handle for the root node
   * \param[in] controller_nh The handle for the controller node
   */
  bool init(LedInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  /** Starts the controller
   *
   * \param[in] time The current time
   */
  void starting(const ros::Time& time) override;

  /** Updates the state of the managed LEDs
   *
   * This method is called periodically by ROS to read/write the state of the LEDs.
   * It publishes a message containing the state off all LEDs on topic 'led_values'
   * and sets the values for LEDs according to the last received command.
   *
   * \param[in] time The current time
   * \param[in] period Unused
   */
  void update(const ros::Time& time, const ros::Duration& /*period*/) override;

  /** Stops the controller
   *
   * \param[in] time Unused
   */
  void stopping(const ros::Time& /*time*/) override;

private:
  typedef boost::shared_ptr<realtime_tools::RealtimePublisher<cooperative_driving_hardware::LedValues>>
      RtPublisherPtr;  ///< Helper definition for realtime_pub_

  /** Data structure to temporally store the values of the last recently received
   * cooperative_driving_hardware::LedCommand in command_ */
  struct Command
  {
    ros::Time stamp;                                    ///< The creation time of a Command
    std::vector<std::pair<std::string, float>> values;  ///< The values of the last recently received
                                                        /// cooperative_driving_hardware::LedCommand

    Command() : stamp(0.0)
    {
    }
  };

  /** Call-back for sub_command_
   *
   * Writes the last recently received cooperative_driving_hardware::LedCommand into command_.
   *
   * \param[in] command The received cooperative_driving_hardware::LedCommand message
   */
  void cmdLedCallback(const cooperative_driving_hardware::LedCommand& command);
  void switch_off();  ///< Helper function for switching off all LEDs

  std::string name_;                                /** The name of the controller */
  std::vector<LedHandle> leds_;                     /** Led handles managed by this controller */
  RtPublisherPtr realtime_pub_;                     /** LedValues publisher */
  ros::Time last_publish_time_;                     /** The last time a LedValues-message was published */
  ros::Rate publish_rate_;                          /** The rate at which LedValues are published */
  std::string base_frame_id_;                       /** The identifier of the base frame */
  double cmd_timeout_;                              /** Time in seconds after which LedCommand(s) are considered old */
  realtime_tools::RealtimeBuffer<Command> command_; /** Last (received) Command */
  ros::Subscriber sub_command_;                     /** Subscriber for LedCommands */
};

}  // namespace cooperative_driving

#include "cooperative_driving/disable_ros_warnings_pre.h"
PLUGINLIB_EXPORT_CLASS(cooperative_driving::LedController, controller_interface::ControllerBase)
#include "cooperative_driving/disable_ros_warnings_post.h"
