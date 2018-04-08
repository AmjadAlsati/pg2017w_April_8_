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

#include <cmath>
#include <functional>
#include <utility>
#include <vector>

// FIXME: Get rid of dependency on ros message definition since this is the only ros-related dependency here
#include <geometry_msgs/Twist.h>

#include "apriltags2_ros/AprilTagDetection.h"

#include "cooperative_driving/logic.h"
#include "cooperative_driving/moment.h"
#include "cooperative_driving/pid_controller.h"
#include "cooperative_driving/util.h"
#include "cooperative_driving/disable_ros_warnings_pre.h"
#include "cooperative_driving_vision/Features.h"
#include "cooperative_driving/disable_ros_warnings_post.h"

namespace cooperative_driving
{
/**   Enumeration of all simple behaviours that are available */
enum SimpleBehavior
{
  SIMPLE_FOLLOW_LINE = 0,
  SIMPLE_KEEP_BLOB_DISTANCE,
  SIMPLE_TURN_TO_BLOB,
  SIMPLE_REMOTE_SPEED,
  SIMPLE_REMOTE_STEERING,
  SIMPLE_STATIC_VELOCITY,
  SIMPLE_BEHAVIOR_L
};

/** Computes instructions to keep a detected line in front of the robot.
 *
 * Computes instructions to keep a detected line in front of the robot.
 * Uses a PID-controller to dampen the result.
 *
 * \param[in] pid Reference to the PID controller that is used.
 * \param[in] line_location The current line location.
 * \param[in] image_width Number of columns in the image the line was extracted from.
 * \param[in] k_p The PID-controller's proportional component.
 * \param[in] k_i The PID-controller's integral component.
 * \param[in] k_d The PID-controller's derivative component.
 * \param[in] time_interval The time difference to the last iteration.
 *
 * \return The VelocityVector to use to keep the line in front of the robot.
 */
const VelocityVector follow_line(PIDController &pid, const double line_location, const unsigned image_width,
                                 const double k_p, const double k_i, const double k_d, const double time_interval)
{
  const auto normalized_location = -1.0 * (clamp(line_location / image_width, 0.0, 1.0) - 0.5) * 2.0;
  const double steer = pid(normalized_location, time_interval, k_p, k_i, k_d);
  ROS_DEBUG("[Follow Line] steer: %f", steer);
  return { 0, steer };
}

/** The resulting vector instruction maintain a constant distance to the AprilTag.
 *
 * Computes a vector that, when used to control the robots velocity, maintains a fixed
 * distance to the AprilTag. Depending on the offset
 *
 * \param[in] pid             Reference to the PID controller instance that is used
 * \param[in] apriltag        The target tag that should be followed.
 * \param[in] image_width     Number of columns in the image the blob was extracted from.
 * \param[in] k_p             The PID-controller's proportional component.
 * \param[in] k_i             The PID-controller's integral component.
 * \param[in] k_d             The PID-controller's derivative component.
 * \param[in] time_interval   The time difference to the last iteration.
 * \param[in] deadzone        Tolerance interval around target_distance where velocity is zero.
 *
 * \return The VelocityVector to use when trying to keep the blob distance.
 */
const VelocityVector keep_tag_distance(PIDController &pid, const apriltags2_ros::AprilTagDetection &apriltag,
                                        const unsigned image_width, const double k_p, const double k_i,
                                        const double k_d, const double time_interval, const double deadzone)
{
  const double distance = apriltag.pose.pose.pose.position.z * 100; // CAREFUL: distance is in cm, z is in m
  const double speed = -1.0 * pid(distance, time_interval, k_p, k_i, k_d);
  ROS_DEBUG("[Keep Blob Distance] Blob distance: %f ,PID value: %f ,Deadzone: %f, Result: %f", distance, speed,
            deadzone, (fabs(speed) <= deadzone ? 0 : speed));
  return { (fabs(speed) <= deadzone ? 0 : speed), 0 };
  return { 0, 0 };
}

/** The resulting instruction will turn to the blob.
 *
 * \param[in] blob            The target blob.
 * \param[in] image_width     Number of columns in the image the blob was extracted from.
 *
 * \return The VelocityVector to use when trying to turn to the detected blob.
 */
constexpr const VelocityVector turn_to_blob(const cooperative_driving_vision::Region &blob, const unsigned image_width)
{
  // TODO: Adapt for AprilTags? This function apparently isn't being used anywhere.
  return { 0, (clamp(static_cast<float>((blob.moment.m10 / blob.moment.m00) / image_width), 0.f, 1.f) - 0.5f) * 2.0f };
}

/** The resulting instruction will set the speed based on an external command.
 *
 * \param[in] command  The received command.
 *
 * \return The VelocityVector to use when using remote control speed command.
 */
const VelocityVector remote_speed(const geometry_msgs::TwistConstPtr &command)
{
  return { clamp(command->linear.x, -1., 1.), 0 };
}

/** The resulting instruction will set the steering based on an external command.
 *
 * \param[in] command  The received command.
 *
 * \return The VelocityVector to use when using remote control steering command.
 */
const VelocityVector remote_steering(const geometry_msgs::TwistConstPtr &command)
{
  return { 0, clamp(command->angular.z, -1., 1.) };
}

/** Instruction for going forward with a static velocity.
 *
 * \return The VelocityVector to use when driving with a static velocity.
 */
constexpr const VelocityVector static_velocity()
{
  return { 1, 0 };
}

}  // namespace cooperative_driving
