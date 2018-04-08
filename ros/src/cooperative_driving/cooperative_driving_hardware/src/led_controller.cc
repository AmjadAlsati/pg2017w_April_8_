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
#include "cooperative_driving/led_controller.h"

#include "cooperative_driving/util.h"

namespace cooperative_driving
{
bool LedController::init(LedInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  const std::string complete_ns = controller_nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  name_ = complete_ns.substr(id + 1);

  // get publishing period
  double publish_rate;
  if (!controller_nh.getParam("publish_rate", publish_rate))
  {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }
  publish_rate_ = publish_rate;

  if (!controller_nh.getParam("cmd_timeout", cmd_timeout_))
  {
    ROS_ERROR("Parameter 'cmd_timeout' not set");
    return false;
  }

  ROS_INFO_STREAM_NAMED(name_, "Led commands will be considered old if they are older than " << cmd_timeout_ << "s.");
  controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

  // get all joint states from the hardware interface
  for (const auto& led_name : hw->getNames())
  {
    ROS_DEBUG("Got led %s", led_name.c_str());

    // led handle
    leds_.push_back(hw->getHandle(led_name));

    // realtime publisher
    realtime_pub_ = RtPublisherPtr(
        new realtime_tools::RealtimePublisher<cooperative_driving_hardware::LedValues>(root_nh, "/led_controller/led_values", 2));
  }

  sub_command_ = controller_nh.subscribe("/led_controller/cmd_led", 1, &LedController::cmdLedCallback, this);

  return true;
}

void LedController::starting(const ros::Time& time)
{
  switch_off();
  publish_rate_.reset();
  last_publish_time_ = time;
}

void LedController::update(const ros::Time& time, const ros::Duration& /*period*/)
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

      // Value
      for (const auto& led : leds_)
      {
        realtime_pub_->msg_.values.push_back(led);
      }
      realtime_pub_->unlockAndPublish();
    }
  }

  // set led values
  Command current_command = *(command_.readFromNonRT());
  const double dt = (time - current_command.stamp).toSec();

  if (dt <= cmd_timeout_)
  {
    for (const auto& led_value : current_command.values)
    {
      const auto it = std::find_if(leds_.begin(), leds_.end(), [&led_value](const LedHandle& handle) {
        return handle.getFrameId() == led_value.first;
      });
      if (it != leds_.end())
      {
        it->setValue(led_value.second);
      }
    }
  }
  else
  {
    //    switch_off();
  }
}

void LedController::cmdLedCallback(const cooperative_driving_hardware::LedCommand& command)
{
  if (isRunning())
  {
    Command command_struct;
    command_struct.stamp = ros::Time::now();

    command_struct.values.resize(command.values.size());
    std::transform(command.values.begin(), command.values.end(), command_struct.values.begin(),
                   [](const cooperative_driving_hardware::LedValue& led_value) {
                     return std::make_pair(led_value.frame_id, clamp(led_value.value, 0.f, 1.f));
                   });

    command_.writeFromNonRT(command_struct);
  }
  else
  {
    ROS_ERROR_NAMED(name_, "Can't accept new commands. LedController is not running.");
  }
}

void LedController::stopping(const ros::Time& /*time*/)
{
  switch_off();
}

void LedController::switch_off()
{
  // turn off leds
  for (auto& led : leds_)
  {
    led.setValue(0);
  }
}

}  // namespace cooperative_driving
