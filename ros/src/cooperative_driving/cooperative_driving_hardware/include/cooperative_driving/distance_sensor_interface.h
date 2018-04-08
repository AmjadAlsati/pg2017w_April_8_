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
#include "cooperative_driving_hardware/DistanceValue.h"
#include "cooperative_driving/disable_ros_warnings_post.h"

namespace cooperative_driving
{
/** \brief A handle used to read the state of a CCS distance sensor. */
class DistanceSensorHandle
{
public:
  /** Constructor of DistanceSensorHandle
   *
   * \param[in] name The value for DistanceHandle::name_
   * \param[in] distance The value for DistanceHandle::distance_
   */
  DistanceSensorHandle(const std::string& name, float* distance) : name_(name), frame_id_(name), distance_(distance)
  {
  }

  /** Return the name of the sensor
   *
   * \retun The value of DistanceSensorHandle::name_
   */
  inline std::string getName() const
  {
    return name_;
  }

  /** Return the frame identifier of the sensor
   *
   * \return The value of DistanceSensorHandle::frame_id_
   */
  inline std::string getFrameId() const
  {
    return frame_id_;
  }

  /** Return the current value of the distance reading
   *
   * \return The current value of DistanceSensorHandle::distance_
   */
  inline float getDistance() const
  {
    return *distance_;
  }

  /** Constructs a cooperative_driving_hardware::DistanceValue message for this sensor
   *
   * \return A cooperative_driving_hardware::DistanceValue message containing the current state of the sensor
   */
  inline operator cooperative_driving_hardware::DistanceValue() const
  {
    cooperative_driving_hardware::DistanceValue result;
    result.frame_id = getFrameId();
    result.distance = getDistance();
    return result;
  }

private:
  std::string name_;     /** The name of the sensor */
  std::string frame_id_; /** The frame identifier of the sensor */
  float* distance_;      /** Storage location for the current reading of the sensor */
};

/** \brief Hardware interface to support reading the state of an CCS distance sensor. */
class DistanceSensorInterface : public hardware_interface::HardwareResourceManager<DistanceSensorHandle>
{
};
}
