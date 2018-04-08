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
#include "cooperative_driving/ccs_robot.h"

#include <cstdint>
#include <exception>

#include "cooperative_driving/disable_ros_warnings_pre.h"
#include <controller_manager/controller_manager.h>
#include "cooperative_driving/disable_ros_warnings_post.h"

#include <cstdio>

extern "C" {
#include "cooperative_driving/ccs_hw.h"
}

namespace cooperative_driving
{
CCSRobot::CCSRobot(const std::string& i2c_device, double ros_hw_velocity_conversion_factor,
                   uint8_t ros_hw_led_max_value, uint8_t distance_sensor_index)
  : jnt_state_interface_()
  , jnt_vel_interface_()
  , distance_sensor_interface_()
  , led_interface_()
  , cmd_vel_{ 0, 0 }
  , cmd_led_{ 0, 0 }
  , pos_{ 0, 0 }
  , vel_{ 0, 0 }
  , eff_{ 0, 0 }
  , dist_{ -1, -1, -1, -1, -1 }
  , led_{ 0, 0 }
  , i2c_device_(ccs_open(i2c_device.c_str()))
  , ros_hw_velocity_conversion_factor_(ros_hw_velocity_conversion_factor)
  , ros_hw_led_max_value_(ros_hw_led_max_value)
  , distance_sensor_index_(distance_sensor_index)
{
  if (i2c_device_ < 0)
  {
    throw std::runtime_error("Failed to setup i2c device");
  }

  // register state interface for the wheels
  jnt_state_interface_.registerHandle(
      hardware_interface::JointStateHandle("left_wheel_hinge", &pos_[0], &vel_[0], &eff_[0]));
  jnt_state_interface_.registerHandle(
      hardware_interface::JointStateHandle("right_wheel_hinge", &pos_[1], &vel_[1], &eff_[1]));
  registerInterface(&jnt_state_interface_);

  // register command interface for the wheels
  jnt_vel_interface_.registerHandle(
      hardware_interface::JointHandle(jnt_state_interface_.getHandle("left_wheel_hinge"), &cmd_vel_[0]));
  jnt_vel_interface_.registerHandle(
      hardware_interface::JointHandle(jnt_state_interface_.getHandle("right_wheel_hinge"), &cmd_vel_[1]));
  registerInterface(&jnt_vel_interface_);

  // register interface for distance sensors
  /* Use only one sensor */
  distance_sensor_interface_.registerHandle(DistanceSensorHandle("front_center", &dist_[distance_sensor_index]));
  registerInterface(&distance_sensor_interface_);

  // register interface for leds
  led_interface_.registerHandle(LedHandle("led/red", &cmd_led_[0], &led_[0]));
  led_interface_.registerHandle(LedHandle("led/blue", &cmd_led_[1], &led_[1]));
  registerInterface(&led_interface_);
}

CCSRobot::~CCSRobot()
{
  ccs_close(i2c_device_);
}

void CCSRobot::read(const ros::Time& /*time*/, const ros::Duration& period)
{
  int8_t left_velocity, right_velocity;
  if (ccs_read_velocities(i2c_device_, &left_velocity, &right_velocity) < 0)
  {
    ROS_ERROR_STREAM("Failed to read velocity from microcontroller");
    return;
  }
  vel_[0] = left_velocity / ros_hw_velocity_conversion_factor_;
  vel_[1] = right_velocity / ros_hw_velocity_conversion_factor_;

  pos_[0] += vel_[0] * period.toSec();
  pos_[1] += vel_[1] * period.toSec();

  Sensors sensors;
  if (ccs_read_sensors(i2c_device_, &sensors) < 0)
  {
    ROS_ERROR_STREAM("Failed to read distance values from microcontroller");
    return;
  }
  dist_[0] = (sensors.analog[1] <= 80) ? (static_cast<float>(sensors.analog[1]) / 100) : -1;
  dist_[1] = (sensors.analog[2] <= 80) ? (static_cast<float>(sensors.analog[2]) / 100) : -1;
  dist_[2] = (sensors.analog[3] <= 80) ? (static_cast<float>(sensors.analog[3]) / 100) : -1;
  dist_[3] = (sensors.analog[4] <= 80) ? (static_cast<float>(sensors.analog[4]) / 100) : -1;
  dist_[4] = (sensors.analog[5] <= 80) ? (static_cast<float>(sensors.analog[5]) / 100) : -1;

  uint8_t red, blue;
  if (ccs_read_led(i2c_device_, &red, &blue) < 0)
  {
    ROS_ERROR_STREAM("Failed to read led values from microcontroller");
    return;
  }
  led_[0] = static_cast<uint8_t>(red / ros_hw_led_max_value_);
  led_[1] = static_cast<uint8_t>(blue / ros_hw_led_max_value_);
}

void CCSRobot::write(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  int8_t left_velocity = cmd_vel_[0] * ros_hw_velocity_conversion_factor_;
  int8_t right_velocity = cmd_vel_[1] * ros_hw_velocity_conversion_factor_;
  ROS_DEBUG_STREAM("Write velocities: " << cmd_vel_[0] << ", " << cmd_vel_[1] << ", "
                                        << static_cast<int16_t>(left_velocity) << ", "
                                        << static_cast<int16_t>(right_velocity));
  if (ccs_write_velocities(i2c_device_, left_velocity, right_velocity) < 0)
  {
    ROS_ERROR_STREAM("Failed to write velocity command to microcontroller");
    return;
  }
  // led stuff
  uint8_t red = static_cast<uint8_t>(cmd_led_[0] * ros_hw_led_max_value_);
  uint8_t blue = static_cast<uint8_t>(cmd_led_[1] * ros_hw_led_max_value_);

  if (ccs_write_led(i2c_device_, red, blue) < 0)
  {
    ROS_ERROR_STREAM("Failed to write led command to microcontroller");
    return;
  }
}

float CCSRobot::get_measured_distance() const
{
  return dist_[distance_sensor_index_];
}

}  // namespace cooperative_driving
