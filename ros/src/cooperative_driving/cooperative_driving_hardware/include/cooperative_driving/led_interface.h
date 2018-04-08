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

#include <string>

#include <hardware_interface/internal/hardware_resource_manager.h>

#include "cooperative_driving/disable_ros_warnings_pre.h"
#include "cooperative_driving_hardware/LedValue.h"
#include "cooperative_driving/disable_ros_warnings_post.h"

namespace cooperative_driving
{
/** \brief A handle used to read and write the state of a LED */
class LedHandle
{
public:
  /** Constructor of LedHandle
   *
   * \param[in] name The value for LedHandle::name_
   * \param[in] cmd_value The value for LedHandle::cmd_value_
   * \param[in] value The value for LedHandle::value_
   */
  LedHandle(const std::string& name, float* cmd_value, float* value)
    : name_(name), frame_id_(name), cmd_value_(cmd_value), value_(value)
  {
  }

  /** Return the name of the LED
   *
   * \return The value of LedHandle::name_
   */
  inline std::string getName() const
  {
    return name_;
  }

  /** Return the frame identifier of the LED
   *
   * \return The value of LedHandle::frame_id_
   */
  inline std::string getFrameId() const
  {
    return frame_id_;
  }

  /** Sets a new value to set the LED to
   *
   * \param[in] value A new value for LedHandle::cmd_value_
   */
  void setValue(float value)
  {
    *cmd_value_ = value;
  }

  /** Return the current value of the LED
   *
   * \return The current value of LedHandle::value_
   */
  inline float getValue() const
  {
    return *value_;
  }

  /** Constructs a cooperative_driving_hardware::LedValue message for this LED
   *
   * \return A cooperative_driving_hardware::LedValue message containing the current state of the LED
   */
  inline operator cooperative_driving_hardware::LedValue() const
  {
    cooperative_driving_hardware::LedValue result;
    result.frame_id = getFrameId();
    result.value = getValue();
    return result;
  }

private:
  std::string name_;     /** The name of the LED */
  std::string frame_id_; /** The frame identifier of the LED */
  float* cmd_value_;     /** A variable in the robot for defining the value to set the LED to */
  float* value_;         /** A variable in the robot for storing current value of the LED */
};

/** \brief Hardware interface to support reading and writing the state of a LED. */
class LedInterface : public hardware_interface::HardwareResourceManager<LedHandle>
{
};
}
