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
#include "cooperative_driving/distance_sensor_controller.h"

namespace cooperative_driving
{
bool DistanceSensorController::init(cooperative_driving::DistanceSensorInterface* hw, ros::NodeHandle& root_nh,
                                    ros::NodeHandle& controller_nh)
{
  // get publishing period
  double publish_rate;
  if (!controller_nh.getParam("publish_rate", publish_rate))
  {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }
  publish_rate_ = publish_rate;

  // get all joint states from the hardware interface
  for (const auto& sensor_name : hw->getNames())
  {
    ROS_DEBUG("Got sensor %s", sensor_name.c_str());

    // sensor handle
    sensors_.push_back(hw->getHandle(sensor_name));
  }
  // realtime publisher
  realtime_pub_ = RtPublisherPtr(
      new realtime_tools::RealtimePublisher<cooperative_driving_hardware::DistanceValues>(root_nh, "/distance_controller/distances", 2));
  return true;
}

void DistanceSensorController::starting(const ros::Time& time)
{
  publish_rate_.reset();
  last_publish_time_ = time;
}

void DistanceSensorController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  using namespace hardware_interface;

  // limit rate of publishing
  if (last_publish_time_ + publish_rate_.expectedCycleTime() < time)
  {
    // try to publish
    if (realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + publish_rate_.expectedCycleTime();

      // populate message
      realtime_pub_->msg_.header.stamp = time;
      realtime_pub_->msg_.values.clear();
      // Distance
      for (const auto& sensor : sensors_)
      {
        if (sensor.getDistance() != -1)
        {
          realtime_pub_->msg_.values.push_back(sensor);
        }
      }
      realtime_pub_->unlockAndPublish();
    }
  }
}

}  // namespace cooperative_driving
