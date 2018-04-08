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
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include "cooperative_driving/disable_ros_warnings_post.h"

#include "cooperative_driving/distance_sensor_interface.h"
#include "cooperative_driving/led_interface.h"


namespace cooperative_driving
{

/** Performs all the basic operations on the CCS Robot hardware.
*
* Interacts with Engines, Microcontroller, LEDs and distance sensors.
* The operations done on the robot consist of:
* Writing : Engine velocity, turning on/off the LEDs 
* Reading : Engine Velocity, distance sensor values
*/
class CCSRobot : public hardware_interface::RobotHW
{
public:

/** Constructor of the CCSRobot class.
* 
* Performs required device checks and registers ROS interfaces.
* \param[in] i2c_device                         i2c device object
* \param[in] ros_hw_velocity_conversion_factor  conversion factor to scale velocity
* \param[in] ros_hw_led_max_value               roof value for LED state
* \param[in] distance_sensor_index              index of the distance sensor values
*
*/
  CCSRobot(const std::string& i2c_device, double ros_hw_velocity_conversion_factor, uint8_t ros_hw_led_max_value,
           uint8_t distance_sensor_index);

/** Deconstructor of the class. 
*
* Closes the i2c device properly.
*/
  ~CCSRobot();

/** Generic read() call made every `tick` to get an update of the 
* values from the microcontroller. 
*
* Reads the following values at once:
* - Engine Velocities
* - Distance Sensor values
* - LED states
*
* \param[in] time     Ros time object 
* \param[in] period   Ros duration object
*/
  virtual void read(const ros::Time& time, const ros::Duration& period) override;

/** Generic write() call made every `tick` to write the values to the 
* microcontroller. 
*
* Writes the following values at once:
* - Engine Velocities
* - LED state values
*
* \param[in] Ros time object
* \param[in] Ros duration object
*/  
  virtual void write(const ros::Time& time, const ros::Duration& period) override;

/** Returns the measurements read from the distance sensors.
*
* \return Value of distance sensor
*/
  float get_measured_distance() const;

private:
  hardware_interface::JointStateInterface jnt_state_interface_;             //!< Hardware interface to support reading the state of an array of joints
  hardware_interface::VelocityJointInterface jnt_vel_interface_;            //!< Interface for commanding velocity-based joints
  cooperative_driving::DistanceSensorInterface distance_sensor_interface_;  //!< Interface for the distance sensor hardware
  cooperative_driving::LedInterface led_interface_;                         //!< Interface for the LED hardware
  double cmd_vel_[2];                                                       //!< Command for the engine velocities
  float cmd_led_[2];                                                        //!< Command for the LEDs
  double pos_[2];                                                           //!< Used for reading the odometry values from microcontroller
  double vel_[2];                                                           //!< Used for reading the velocity value from the microcontroller
  double eff_[2];                                                           //!< Used in the Joint state interface for hardware configuration of the robot
  float dist_[5];                                                           //!< Used for reading the values from the distance sensors 
  float led_[2];                                                            //!< Used for reading the LED state from the microcontroller
  int i2c_device_;                                                          //!< i2c device for reading and writing operations
  const double ros_hw_velocity_conversion_factor_;                          //!< Factor to convert code velocity to writable hardware velocity
  const uint8_t ros_hw_led_max_value_;                                      //!< Roof value for the LED state
  int distance_sensor_index_;                                               //!< Index to iterate distance sensor values
};

}  // namespace cooperative_driving
