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
#include <memory>
#include <thread>

#include "cooperative_driving/disable_ros_warnings_pre.h"
#include <controller_manager/controller_manager.h>
#include <geometry_msgs/TwistStamped.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "cooperative_driving/disable_ros_warnings_post.h"

#include "cooperative_driving/ccs_robot.h"

#include "cooperative_driving/disable_ros_warnings_pre.h"
#include "cooperative_driving_networking/EmergencyBrake.h"
#include "cooperative_driving/disable_ros_warnings_post.h"

namespace cooperative_driving
{

/** This is the running instance of the "robot", as a nodelet. 
*
* It is responsible for regularly executing the read() and the write() calls based on a clock to 
* perform the sensory and the actuation activities. It also takes care of publishing the emergency
* braking messages when needed and actuating a brake by publishing zero velocity message.
*
* Published Topics:
*   messages/emergency_brake : Emergency braking messages.
*   drive_controller/cmd_vel : Publish zero velocity messages
*/
class CCSRobotNodelet : public nodelet::Nodelet
{
public:

/* Constructor of CCSRobotNodelet class.
*
* Calls the required initialisation functions. 
*/
  CCSRobotNodelet() : robot_(), timer_()
  {
  }

/** Deconstructor of the CCSRobotNodelet class.
*
* Stops the timer.
*/

  ~CCSRobotNodelet()
  {
    timer_.stop();
  }

/** The initialisation call that takes care of setting up stuff.
*
* Sets up the node handle, gets the parameters, sets up the tick rate and creates the publisher for emergency messages,  
*
*/
  void onInit() override
  {
    auto nh = getMTNodeHandle();
    const auto private_nh = getPrivateNodeHandle();

    double ros_hw_velocity_conversion_factor;
    int ros_hw_led_max_value;
    int distance_sensor_index;
    std::string i2c_device;
    private_nh.getParam("ros_hw_velocity_conversion_factor", ros_hw_velocity_conversion_factor);
    private_nh.getParam("ros_hw_led_max_value", ros_hw_led_max_value);
    private_nh.param("distance_sensor_index", distance_sensor_index, 0);
    private_nh.param("i2c_device", i2c_device, std::string("/dev/i2c-1"));
    private_nh.param("braking_threshold", braking_threshold_, 0.0);

    robot_.reset(
        new CCSRobot(i2c_device, ros_hw_velocity_conversion_factor, ros_hw_led_max_value, distance_sensor_index));

    cm_.reset(new controller_manager::ControllerManager(robot_.get(), nh));

    int rate;
    private_nh.getParam("rate", rate);
    timer_ = nh.createTimer(ros::Rate(rate), &CCSRobotNodelet::tick, this);

    // Create publisher for transmitting emergency braking messages
    emergency_brake_pub_ =
        nh.advertise<cooperative_driving_networking::EmergencyBrake>("/robot/emergency_brake", 2);
    cmd_publisher_ = nh.advertise<geometry_msgs::Twist>("/drive_controller/cmd_vel", 1);
  }

/** Callback like function to perform the required operations after appropriate time interval
* 
* \param[in] TimerEvent Ros TimerEvent object
*
* \todo Find better WallDuration to Duration conversion
*/
  void tick(const ros::TimerEvent &timerEvent)
  {
    ros::Duration duration(
        timerEvent.profile.last_duration.sec,
        timerEvent.profile.last_duration.nsec);

    robot_->read(timerEvent.current_real, duration);
    cm_->update(timerEvent.current_real, duration);
    brake_if_necessary();
    robot_->write(timerEvent.current_real, duration);
  }

private:

/** Emergency brake if distance sensor value is below threshold.
*
*/
  void brake_if_necessary()
  {
    
    double distance = robot_->get_measured_distance();
    if (distance > 0 && distance < braking_threshold_)
    {
      brake();

      if (!braking_)
      {
        publish_emergency_brake(true);

        braking_ = true;
      }
    }
    else if (distance >= braking_threshold_ && braking_)
    {
      publish_emergency_brake(false);

      braking_ = false;
    }
  }

/** Send empty messages to have (and keep) robot stopped.
*
*/
  void brake()
  {
    
    cmd_publisher_.publish(geometry_msgs::Twist());
  }

/** Create and publish EmergencyBraking message (true) so that
* other robots stop too.
*
* \param[in] enable Boolean to enable breaking
*
*/
  void publish_emergency_brake(bool enable)
  {
    cooperative_driving_networking::EmergencyBrake msg;
    msg.header.stamp = ros::Time::now();
    msg.enable = enable;
    int sender_id;
    ros::param::get("/demo_app/id", sender_id);
    msg.sender_id = sender_id;

    emergency_brake_pub_.publish(msg);
  }

  std::unique_ptr<CCSRobot> robot_;                             //!< Used for robot object specific operations 
  std::unique_ptr<controller_manager::ControllerManager> cm_;   //!< Used for controller manager object specific operations
  ros::Timer timer_;                                            //!< Timer object used in time calculations eg, ticks
  double braking_threshold_;                                    //!< Emergency breaking threshold, decides when to break
  ros::Publisher emergency_brake_pub_;                          //!< Publisher object to publish emergency breaking messages
  ros::Publisher cmd_publisher_;                                //!< Publisher object to publish the cmd_vel messages
  bool braking_ = false;                                        //!< Used to track the breaking state
};
}  // namespace cooperative_driving

#include "cooperative_driving/disable_ros_warnings_pre.h"
PLUGINLIB_EXPORT_CLASS(cooperative_driving::CCSRobotNodelet, nodelet::Nodelet)
#include "cooperative_driving/disable_ros_warnings_post.h"
