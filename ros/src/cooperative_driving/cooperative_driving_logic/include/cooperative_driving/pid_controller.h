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

#include "cooperative_driving/util.h"

namespace cooperative_driving
{
/** A proportional–integral–derivative controller (PID controller) is a control loop feedback mechanism (controller)
 * commonly used in industrial control systems.
 * 
 * PID-controller is used to control different behaviors and to get rid of oszillation behavior for certain behaviours of
 * the logic behaviours.
 */
struct PIDController
{
public:
  /** Constructs the PID controller
   *
   * \param[in] target_position The target position the controller maintain
   * \param[in] windup_limit Limits the absolute value of the integral term
   * \param[in] upper_limit The upper bound of the PID controller values
   * \param[in] lower_limit The lower bound of the PID controller values
   */
  PIDController(double target_position, double windup_limit = 1000, double upper_limit = 1000,
                double lower_limit = 1000)
    : previous_error_(0)
    , previous_integral_(0)
    , target_position_(target_position)
    , windup_limit_(windup_limit)
    , upper_limit_(upper_limit)
    , lower_limit_(lower_limit)
  {
  }

  /** Changes the target position of the current pid instance.
   *
   * \param target_position The new target position to set
   */
  void change_target(double target_position)
  {
    target_position_ = target_position;
  }

  /** Changes the windup limit for the integral part of the current pid instance.
   *
   * \param windup_limit The new windup distance to set
   */
  void change_windup_limit(double windup_limit)
  {
    windup_limit_ = windup_limit;
  }

  /** Resets the pid controller. Values for previous error and integral are reseted zo zero.
   */
  void reset()
  {
    previous_error_ = 0.0;
    previous_integral_ = 0.0;
  }

  /** Computes the value with regard to last error and the current integral term.
   *
   * \param current_location The current location.
   * \param time_interval The time difference between this and the last iteration.
   * \param k_p The PID-controller's proportional component.
   * \param k_i The PID-controller's integral component.
   * \param k_d The PID-controller's derivative component.
   */
  double operator()(const double current_location, const double time_interval, const double k_p, const double k_i,
                    const double k_d)
  {
    // P
    double error = target_position_ - current_location;
    double P = k_p * error;

    // D
    double change_rate = (error - previous_error_) / time_interval;
    double D = k_d * change_rate;

    // I
    double integral = clamp(previous_integral_ + error, -1.0 * windup_limit_, windup_limit_);
    double I = k_i * integral;

    previous_error_ = error;
    previous_integral_ = integral;

    return clamp(P + I + D, lower_limit_, upper_limit_);
  }

private:
  double previous_error_;    //!< The PID-controller's last error measurement.
  double previous_integral_; //!< The PID-controller's last integral measurement.
  double target_position_;   //!< The PID-controller's target position.
  double windup_limit_;      //!< Anti-windup term. Limits the absolute value of the integral term.
  double upper_limit_;       //!< Upper saturation limit.
  double lower_limit_;       //!< Lower saturation limit.
};

}  // namespace cooperative_driving
