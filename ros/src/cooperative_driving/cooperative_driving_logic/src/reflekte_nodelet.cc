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
#include <algorithm>
#include <iterator>
#include <tuple>
#include <vector>
#include <string>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdlib>
#include<string>
#include<sstream>
#include<cctype>

#include "cooperative_driving/disable_ros_warnings_pre.h"
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <cv_bridge/cv_bridge.h>
#include "apriltags2_ros/AprilTagDetectionArray.h"
#include "cooperative_driving/disable_ros_warnings_post.h"

#include "cooperative_driving_msgs/Directions.h"
#include "cooperative_driving_msgs/Direction.h"
#include "cooperative_driving_msgs/Command.h"

#include "cooperative_driving/logic.h"
#include "cooperative_driving/pid_controller.h"
#include "cooperative_driving/simple_behaviors.h"
#include "cooperative_driving/reflekte_state_machine.h"
#include "cooperative_driving/vision_util.h"
#include "cooperative_driving_logic/ChangeState.h"
#include "cooperative_driving_logic/ReflekteServerConfig.h"
#include "cooperative_driving_logic/ReflekteState.h"
#include "cooperative_driving_networking/EmergencyBrake.h"
#include "cooperative_driving_vision/Features.h"

#include "cooperative_driving/pid_controller.h"

using cooperative_driving_logic::ChangeState;
using cooperative_driving_networking::EmergencyBrakeConstPtr;

namespace cooperative_driving
{
/** Low-level-logic nodelet computing the correct velocity commands.
 *
 * This Nodelet subscribes to published messages and calculates a corresponding velocity-
 * command which it publishes. The result depends on the robot's current state to allow
 * for different behavior, e.g. remote control and autonomous driving.
 */
class ReflekteNodelet : public nodelet::Nodelet
{
public:
  /** Constructs the ReflekteNodelet.
   */
  ReflekteNodelet()
    : last_command_time_(ros::Time::now())
    , tag_id_to_follow(0) // TODO: make this configurable etc. (see definition of this variable)!
    , tag_to_follow_detected(false)
    , line_following_pid_(0, 10, 1.0, -1.0)
    , blob_distance_pid_(0, 10, 1.0, -1.0)
  {

	Initially_configed = false;// The logic node is running for the first time and it should load the values from the dump file
  }

  /** Destructs the ReflekteNodelet.
   */
  ~ReflekteNodelet()
  {
  }

  /** Initialize the ReflekteNodelet.
   *
   * Subscribes to relevant topics and and advertises the velocity commands.
   */
  void onInit() override
  {
    auto nh = getNodeHandle();
    auto private_nh = getPrivateNodeHandle();
 

if(nh.getParam("reflekte/Dump_Path",DumpPath))//get the path of the dump file in DumpPath attribute from the roslaunch file
{
ROS_INFO("Path to the dump file is: %s",DumpPath.c_str());
}
else
{
ROS_ERROR("Failed to load file Param path 'Dump_Path'");
}


    // start dynamic reconfigure server
    reconfigure_server_.reset(new ReconfigureServer(private_nh));
    reconfigure_server_->setCallback(boost::bind(&ReflekteNodelet::parameterServerCallback, this, _1, _2));

    cmd_publisher_ = nh.advertise<geometry_msgs::Twist>("/drive_controller/cmd_vel", 1);
    feature_subscriber_ = nh.subscribe("/feature_extractor/features", 1, &ReflekteNodelet::onFeatures, this);
    tag_subscriber_ = nh.subscribe("/tag_detections", 1, &ReflekteNodelet::onAprilTags, this);
    remote_control_subscriber_ = nh.subscribe("/remote_control/cmd_vel", 1, &ReflekteNodelet::onRemoteCommand, this);

    crossing_publisher_ = nh.advertise<cooperative_driving_msgs::Directions>("/crossing/directions", 1);
    // Subscribe to DirectionCommand sent by cooperative_driving_application 
    dir_subscriber_ = nh.subscribe("next_action", 1, &ReflekteNodelet::onDirectionCommand, this); 

    change_state_service_ = private_nh.advertiseService("change_state", &ReflekteNodelet::onChangeStateRequest, this);

    state_publisher_ = private_nh.advertise<cooperative_driving_logic::ReflekteState>("current_state", 1, true);
    publish_state();
    stateMachine = createStateMachine();
  }

  /** Create a new State Machine.
   *
   * Creates a state machine and lso creates and adds all needed states.
   *
   * \return The state machine.
   */
  ReflekteStateMachine createStateMachine(){
    State* emergencyState = new State("Emergency State");

    State* turnState = new State("Turn State");

    State* prepareState = new State("Prepare State");
    prepareState->addNextState(emergencyState);

    State* idleState = new State("Idle State");
    idleState->addNextState(emergencyState);

    emergencyState->addNextState(idleState);

    State* lineFollowingState = new State("Line Following");
    lineFollowingState->addNextState(prepareState);
    lineFollowingState->addNextState(emergencyState);
    lineFollowingState->addNextState(idleState);

    turnState->addNextState(idleState);
    turnState->addNextState(lineFollowingState);
    turnState->addNextState(prepareState);
    idleState->addNextState(idleState);
    idleState->addNextState(lineFollowingState);
    prepareState->addNextState(turnState);
    prepareState->addNextState(idleState);

    ReflekteStateMachine stateMachine = ReflekteStateMachine();
    //stateMachine.setStartState(lineFollowingState); // TODO: set start state to ready, after application messages are
                                                    // are received correctly
    stateMachine.setStartState(idleState); // TODO: set start state to ready, after application messages are
    // are received correctly
    turnDirection = "STRAIGHT";

    return stateMachine;
  }
  
  void onAprilTags(const apriltags2_ros::AprilTagDetectionArrayConstPtr &tag_msg)
  {
    // Sort tags by distance:
    /*sort(tag_msg.detections.begin(), tag_msg.detections.end(),
         [](const apriltags2_ros::AprilTagDetection & a, const apriltags2_ros::AprilTagDetection & b) -> bool { 
      return a.pose.pose.pose.position.z < b.pose.pose.pose.position.z;
    });*/
    
    this->tag_to_follow_detected = false;
    for (auto tag_detection : tag_msg->detections) {
      if (tag_detection.id[0] == this->tag_id_to_follow) {
        std::cout << "### Found the AprilTag to follow: ID " << this->tag_id_to_follow << std::endl;
        this->tag_to_follow = tag_detection;
        this->tag_to_follow_detected = true;
        break;
      }
    }
  }

  /** Callback for new features.
   *
   * Computes and publishes a velocity command based on the currently received features.
   * Intermediate results are combined to different possible outputs, one of which is
   * then selected to be published.
   *
   * \param[in] feature_msg The current feature message of the cooperative_driving::FeatureExtractionNodelet
   *
   * \see selectOutput
   * \see sendCommand
   */
  void onFeatures(const cooperative_driving_vision::FeaturesConstPtr &feature_msg)
  {
    double message_duration = (ros::Time::now() - last_command_time_).toSec();
    
    cv::Point2f crossing;
    bool contains_intersection = false;
    bool isValidCrossing = true;
    if (feature_msg->Vlines.size() > 1 && feature_msg->Hlines.size() > 1) {
      cv::Point2f p1_v = cv::Point2f(feature_msg->Vlines.at(0).x, feature_msg->Vlines.at(0).y);
      cv::Point2f p2_v = cv::Point2f(feature_msg->Vlines.at(feature_msg->Vlines.size()-1).x, feature_msg->Vlines.at(feature_msg->Vlines.size()-1).y);
      cv::Point2f p1_h = cv::Point2f(feature_msg->Hlines.at(0).x, feature_msg->Hlines.at(0).y);
      cv::Point2f p2_h = cv::Point2f(feature_msg->Hlines.at(feature_msg->Hlines.size()-1).x, feature_msg->Hlines.at(feature_msg->Hlines.size()-1).y);
  
      contains_intersection = containsIntersections(p1_v, p2_v, p1_h, p2_h, crossing);

      if (contains_intersection) {
        std::vector<cooperative_driving_msgs::Direction>possibleDirectionsVector = CalculateCrossingDirections(p1_v, p2_v, p1_h, p2_h, crossing);
        bool containsStraight = false;
        for (auto const direction : possibleDirectionsVector) {
          if (direction.direction.compare("STRAIGHT") == 0) {
            containsStraight = true;
          }
        }
        if((possibleDirectionsVector.size() == 1 && containsStraight) || possibleDirectionsVector.size() == 0) {
          isValidCrossing = false;
        }

        if(isValidCrossing){
          cooperative_driving_msgs::Directions directionsMsg;
          ROS_INFO_STREAM("Possible Directions:");
          for (auto const direction : possibleDirectionsVector) {
            directionsMsg.directions.push_back(direction);
            ROS_INFO_STREAM(direction.direction);
          }
          crossing_publisher_.publish(directionsMsg);
        }
      }
    }

    if (stateMachine.getState().name.compare("Line Following") == 0) {
      onLineFollowingState(feature_msg, isValidCrossing, message_duration);
    }

    if (stateMachine.getState().name.compare("Prepare State") == 0) {
      onPrepareState(feature_msg, crossing, contains_intersection, message_duration);
    }

    if (stateMachine.getState().name.compare("Turn State") == 0) {
      onTurnState(message_duration, feature_msg);
    }

    if (stateMachine.getState().name.compare("Idle State") == 0){
      onIdleState();
    }
  }

  std::vector<cooperative_driving_msgs::Direction> CalculateCrossingDirections(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f &crossing) {
    std::vector<cooperative_driving_msgs::Direction> directionsList;
    int verticalThreshold = 5;
    int horizontalThreshold = 15;

    if (std::min(o1.y, p1.y) < crossing.y-verticalThreshold) {
      cooperative_driving_msgs::Direction dir;
      dir.direction = "STRAIGHT";
      directionsList.push_back(dir);
    }
    if (std::min(o2.x, p2.x) < crossing.x-horizontalThreshold) {
      cooperative_driving_msgs::Direction dir;
      dir.direction = "LEFT";
      directionsList.push_back(dir);
    }
    if (std::max(o2.x, p2.x) > crossing.x+horizontalThreshold) {
      cooperative_driving_msgs::Direction dir;
      dir.direction = "RIGHT";
      directionsList.push_back(dir);
    }

    return directionsList;
  }

  /** What to do in the line following state.
   *
   * Calculates and sends the new velocity Vector for the line following 
   *
   * \param[in] feature_msg The current feature message of the cooperative_driving::FeatureExtractionNodelet
   * \param[in] message_duration The current time since the last feature_msg.
   *
   */
  void onLineFollowingState(const cooperative_driving_vision::FeaturesConstPtr &feature_msg, bool isValidCrossing, double message_duration) {

    if(isIdle){
      stateMachine.goToNextState("Idle State");
    }else if(isValidCrossing && turnDirection.compare("STRAIGHT") != 0){
      stateMachine.goToNextState("Prepare State");
    }else{
      if (isLeader) {
        if (feature_msg->Vlines.size() >= 1) {
          VelocityVector followLineVelocityVector = follow_line(line_following_pid_, line_location(line_row_location_, feature_msg->Vlines),
                        feature_msg->image_width, LF_k_p_, LF_k_i_, LF_k_d_, message_duration);
          followLineVelocityVector.velocity = static_velocity().velocity;
          sendCommand(followLineVelocityVector);
        } else {
          sendCommand(VelocityVector());
        }
      } else {
        VelocityVector keepBlobDistanceVelocityVector = this->tag_to_follow_detected ? keep_tag_distance(blob_distance_pid_, this->tag_to_follow, feature_msg->image_width, BD_k_p_, BD_k_i_, BD_k_d_, message_duration, BD_deadzone_) : VelocityVector();
        if (feature_msg->Vlines.size() >= 1) {
          VelocityVector followLineVelocityVector = follow_line(line_following_pid_, line_location(line_row_location_, feature_msg->Vlines),
                        feature_msg->image_width, LF_k_p_, LF_k_i_, LF_k_d_, message_duration);
          VelocityVector accumulatedVV = {keepBlobDistanceVelocityVector.velocity, followLineVelocityVector.turn_velocity};
          sendCommand(accumulatedVV);
        } else if (feature_msg->Hlines.size() >= 1) {
          VelocityVector followLineVelocityVector = follow_line(line_following_pid_, line_location(line_row_location_, feature_msg->Hlines),
                        feature_msg->image_width, LF_k_p_, LF_k_i_, LF_k_d_, message_duration);
          
          VelocityVector accumulatedVV = {keepBlobDistanceVelocityVector.velocity, followLineVelocityVector.turn_velocity};
          sendCommand(accumulatedVV);
        } else {
          sendCommand(VelocityVector());
        }
      }
    }
  }


  /** What to do in the prepare state.
   *
   * Calculates and sends the new velocity Vector for the preperation.
   *
   * \param[in] feature_msg The current feature message of the cooperative_driving::FeatureExtractionNodelet
   * \param[in] crossing The current location of a crossing
   * \param[in] contains_intersection Boolean if a crossing was detected.
   * \param[in] message_duration The current time since the last feature_msg.
   *
   */
  void onPrepareState(const cooperative_driving_vision::FeaturesConstPtr &feature_msg, cv::Point2f crossing, bool contains_intersection, double message_duration) {
    if(isIdle){
      stateMachine.goToNextState("Idle State");
    }else{
      double approachVelocity;
      if(contains_intersection){
        approachVelocity = calculateApproachCrossingVelocity(feature_msg, crossing);
      } else {
        approachVelocity = previousVelocity;
      }

      if(approachVelocity < 0.5 ){
        approachVelocity = 0;
        isTurning = true;
        stateMachine.goToNextState("Turn State");
      }

      previousVelocity = approachVelocity;    
      
      if (isLeader) {
        if (feature_msg->Vlines.size() >= 1) {
          VelocityVector followLineVelocityVector = follow_line(line_following_pid_, line_location(line_row_location_, feature_msg->Vlines),
                          feature_msg->image_width, LF_k_p_, LF_k_i_, LF_k_d_, message_duration);
          followLineVelocityVector.velocity = approachVelocity;
          sendCommand(followLineVelocityVector);
        } else {
          sendCommand(VelocityVector());
        }
      } else {
        VelocityVector keepBlobDistanceVelocityVector = this->tag_to_follow_detected ? keep_tag_distance(blob_distance_pid_, this->tag_to_follow, feature_msg->image_width, BD_k_p_, BD_k_i_, BD_k_d_, message_duration, BD_deadzone_) : VelocityVector();
        if (feature_msg->Vlines.size() >= 1) {
          VelocityVector followLineVelocityVector = follow_line(line_following_pid_, line_location(line_row_location_, feature_msg->Vlines),
                        feature_msg->image_width, LF_k_p_, LF_k_i_, LF_k_d_, message_duration);
          VelocityVector accumulatedVV = {keepBlobDistanceVelocityVector.velocity, followLineVelocityVector.turn_velocity};
          sendCommand(accumulatedVV);
        } else if (feature_msg->Hlines.size() >= 1) {
          VelocityVector followLineVelocityVector = follow_line(line_following_pid_, line_location(line_row_location_, feature_msg->Hlines),
                        feature_msg->image_width, LF_k_p_, LF_k_i_, LF_k_d_, message_duration);
          
          VelocityVector accumulatedVV = {keepBlobDistanceVelocityVector.velocity, followLineVelocityVector.turn_velocity};
          sendCommand(accumulatedVV);
        } else {
          sendCommand(VelocityVector());
        }
      }
    }
  }

  /** Calculate velocity vector for approaching a crossing.
   *
   * \param[in] feature_msg The current feature message of the cooperative_driving::FeatureExtractionNodelet
   * \param[in] crossing The current location of a crossing
   *
   */
  double calculateApproachCrossingVelocity(const cooperative_driving_vision::FeaturesConstPtr &feature_msg, cv::Point2f crossing) {
    return 1-(static_cast<double>(crossing.y))/feature_msg->image_height;
  }

  /** What to do in the turn state.
   *
   * Calculates and sends the new velocity Vector for turning.
   *
   * \param[in] time_interval The current time since the last feature_msg.
   * \param[in] feature_msg The current feature message of the cooperative_driving::FeatureExtractionNodelet
   * 
   *
   */
  void onTurnState(double time_interval, const cooperative_driving_vision::FeaturesConstPtr &feature_msg) {
    if (isTurning) {
      if (turnDirection.compare("LEFT") == 0) {
        const double steer = line_following_pid_(.5, time_interval, LF_k_p_, LF_k_i_, LF_k_d_);
        VelocityVector turnVelocityVector = {.5, steer};
        sendCommand(turnVelocityVector);
      }

      if (turnDirection.compare("RIGHT") == 0)  {
        const double steer = line_following_pid_(-.5, time_interval, LF_k_p_, LF_k_i_, LF_k_d_  );
        VelocityVector turnVelocityVector = {.5, steer};
        sendCommand(turnVelocityVector);
      }

      if (feature_msg->Vlines.size() <= 2) {
        lostVlines = true;
      }

      if (feature_msg->Vlines.size() > 2 && lostVlines) {
        isTurning = false;
        lostVlines = false;
      }
    } else {
      previousVelocity = 1;
      turnDirection = "STRAIGHT";
      stateMachine.goToNextState("Line Following");
    }
  }


  /**
   * Calculates angle between two given points.
   *
   * \param[in] v1 First Point
   * \param[in] v2 Second Point
   */
  float angleBetween(const cv::Point2f &v1, const cv::Point2f &v2)
  {
      float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
      float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

      float dot = v1.x * v2.x + v1.y * v2.y;

      float a = dot / (len1 * len2);

      if (a >= 1.0)
          return 0.0;
      else if (a <= -1.0)
          return M_PI;
      else
          return acos(a); // 0..PI
  }

  /** What to do in the prepare state.
   *
   * Calculates and sends the new velocity Vector for idling.
   *
   */
  void onIdleState() {
    sendCommand(VelocityVector());
    if(!isIdle){
      stateMachine.goToNextState("Line Following");
    }
  }

  /** Check if there is an intersection between two lines.
   *
   * Finds the intersection of two lines, or returns false.
   * The lines are defined by (o1, p1) and (o2, p2).
   *
   * \param[in] o1 First Point
   * \param[in] p1 Second Point
   * \param[in] o2 Third Point
   * \param[in] p2 Fourth Point
   * \param[inout] r Location of the crossing, if there is any
   */
  bool containsIntersections(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f &r)
  {
      cv::Point2f x = o2 - o1;
      cv::Point2f d1 = p1 - o1;
      cv::Point2f d2 = p2 - o2;

      float angle = angleBetween(d1, d2);

      double treshold_lower = M_PI / (static_cast<double>(2)) - 0.25;
      double treshold_upper = M_PI / (static_cast<double>(2)) + 0.25;

      if (angle <=  treshold_lower || angle >= treshold_upper) {
        return false;
      }

      float cross = d1.x*d2.y - d1.y*d2.x;
      if (std::abs(cross) < /*EPS*/1e-16)
          return false;

      double t1 = (x.x * d2.y - x.y * d2.x)/cross;
      r = o1 + d1 * t1;
      return true;
  }

  /** Callback for a new remote control command.
   *
   * Computes and publishes a velocity command based on the currently received remote control command.
   *
   * \param[in] remote_control_msg The current remote control message that is used to calculate the velocity command
   *
   * \see nextCommand
   */
  void onRemoteCommand(const geometry_msgs::TwistPtr &remote_control_msg)
  {
  }

  /** Callback for an incoming DirectionCommand.
   */
  void onDirectionCommand(const cooperative_driving_msgs::Command &dir_command_msg)
  {
    if(dir_command_msg.command.compare("Turn")==0){
      turnDirection=dir_command_msg.turn.direction;
    }
    //dir_command_msg.turn.direction
    //ROS_INFO("bla %s\n", dir_command_msg.turn.direction);
    return;
  }

  /** Callback function of a services call requesting to change the current state.
   *
   * Depending on the requested state the nodelet is changing its internal state. The requested
   * state is responsible for choosing the corresponding behaviour.
   *
   * \param[in] request The requested state
   *
   * \return True if the state has been changed, otherwise false
   */
  bool onChangeStateRequest(ChangeState::Request &request, ChangeState::Response & /*response*/)
  {
    NODELET_INFO_STREAM("Changing state: " << request.target_state);
    if(request.target_state.compare("idle") == 0){
      isIdle = true;
    }
    if(request.target_state.compare("follow_line") == 0){
      isLeader = true;
      isIdle = false;
    }
    if(request.target_state.compare("follow_blob") == 0){
      this->tag_id_to_follow = request.tag_id_to_follow;
      std::cout << "### Now following tag ID " << this->tag_id_to_follow << std::endl;
      isLeader = false;
      isIdle = false;
    }
    publish_state();
    return true;
  }

private:
  typedef cooperative_driving_logic::ReflekteServerConfig Config; /**< Encapsulates the
                                                                     cooperative_drivin_logic::ReflekteServerConfig */
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;  /**< Encapsulate the dynamic_reconfigure::Server */

  /** Create and publish a velocity-command.
   *
   * Creates a velocity-command based on the input data. It will publish
   * the velocity-vectors velocities into the corresponding fields.
   *
   * \param[in] vector  The velocities to publish.
   */
  void sendCommand(const VelocityVector &vector)
  {
    geometry_msgs::Twist cmd;
    cmd.linear.x = clamp(vector.velocity, -1.0, 1.0) * max_velocity_;
    cmd.angular.z = clamp(vector.turn_velocity, -1.0, 1.0) * max_turn_rate_;
    last_command_time_ = ros::Time::now();
    cmd_publisher_.publish(cmd);
  }

  /** Publishes the current state of the reflekte nodelet.
   *
   * Create a cooperative_driving_logic::ReflekteState message and publishes it.
   */
  void publish_state()
  {
    cooperative_driving_logic::ReflekteState state_msg;
    state_publisher_.publish(state_msg);
  }

  /** Callback function of the dynamic parameter server.
   *
   * Whenever this callback function is called the parameters that are dynamically configurable
   * are updated. A few parameters are only updated if the value has changed (e.g. windup limit) for
   * performance reasons. That is because if those values has been changed the PID controller needs
   * to be updated as well.
   *
   * \param[in] config The current configured parameters
   */
 void parameterServerCallback(Config &config, uint32_t /*level*/) 
  { if(Initially_configed==true){ //the logic node is already intilized and we should get configuration from the user
    ROS_INFO("Reconfigure Request: %f %f %f", config.max_velocity, config.max_turn_rate, config.target_distance);
    max_velocity_ = config.max_velocity;
    max_turn_rate_ = config.max_turn_rate;
    line_row_location_ = config.line_row_location;
    LF_k_p_ = config.LF_K_p;
    LF_k_i_ = config.LF_K_i;
    LF_k_d_ = config.LF_K_d;
    LF_windup_limit_ = config.LF_windup_limit;
    BD_k_p_ = config.BD_K_p;
    BD_k_i_ = config.BD_K_i;
    BD_k_d_ = config.BD_K_d;
    BD_deadzone_ = config.BD_deadzone;
    if (BD_windup_limit_ != config.BD_windup_limit)
    {
      BD_windup_limit_ = config.BD_windup_limit;
      blob_distance_pid_.change_windup_limit(BD_windup_limit_);
    }
    if (LF_windup_limit_ != config.LF_windup_limit)
    {
      LF_windup_limit_ = config.LF_windup_limit;
      line_following_pid_.change_windup_limit(LF_windup_limit_);
    }
    // if target distance changes, change target for pid controller too
    if (target_distance_ != config.target_distance)
    {
      target_distance_ = config.target_distance;
      blob_distance_pid_.change_target(target_distance_);
    }

dumpConfigValues(); //dump the parameter values in the dump.yaml file

}
else //the logic node is just being intilized and should load the values of the parameters from the dump.yaml
{

LoadInitialParameter();//load the values from the file
Initially_configed=true;//set this value to false since the logic node is already intilized now
}
ROS_INFO("velocity is %f \n", max_velocity_); //for debug purpose
ROS_INFO("max_turn_rate_ is %f \n", max_turn_rate_); //for debug purpose
ROS_INFO("line_row_location_ is %d \n", line_row_location_); //for debug purpose
ROS_INFO("LF_k_p_ is %f \n", LF_k_p_); //for debug purpose
ROS_INFO("LF_k_i_ is %f \n", LF_k_i_); //for debug purpose
ROS_INFO("LF_k_d_ is %f \n", LF_k_d_); //for debug purpose
ROS_INFO("LF_windup_limit_ is %f \n", LF_windup_limit_); //for debug purpose
ROS_INFO("BD_k_p_ is %f \n", BD_k_p_); //for debug purpose
ROS_INFO("BD_k_i_ is %f \n", BD_k_i_); //for debug purpose
ROS_INFO("BD_k_d_ is %f \n", BD_k_d_); //for debug purpose
ROS_INFO("BD_windup_limit_ is %f \n", BD_windup_limit_); //for debug purpose
ROS_INFO("BD_deadzone_ is %f \n", BD_deadzone_); //for debug purpose







  }

void LoadInitialParameter() //load all the logic parameters from dump.yaml

{
    max_velocity_ =std::stod( extractNumber("max_velocity",DumpPath),nullptr);
    max_turn_rate_ = std::stod(extractNumber("max_turn_rate",DumpPath),nullptr);
    line_row_location_ = std::stoi( extractNumber("line_row_location",DumpPath),nullptr);
    LF_k_p_ = std::stod(extractNumber("LF_K_p",DumpPath),nullptr);
    LF_k_i_ = std::stod(extractNumber("LF_K_i",DumpPath),nullptr);
    LF_k_d_ = std::stod(extractNumber("LF_K_d",DumpPath),nullptr);
    LF_windup_limit_ = std::stod(extractNumber("LF_windup_limit",DumpPath),nullptr);
    BD_k_p_ = std::stod(extractNumber("BD_K_p",DumpPath),nullptr);
    BD_k_i_ = std::stod(extractNumber("BD_K_i",DumpPath),nullptr);
    BD_k_d_ = std::stod(extractNumber("BD_K_d",DumpPath),nullptr);
    BD_deadzone_ = std::stod(extractNumber("BD_deadzone",DumpPath),nullptr);
    BD_windup_limit_ = std::stod(extractNumber("BD_windup_limit",DumpPath),nullptr);
    if(max_velocity_ < 0 || max_turn_rate_<0 || line_row_location_ < 0 || LF_k_p_ <0 || LF_k_i_ <0 || LF_k_d_ <0 || LF_windup_limit_<0 ||BD_k_p_<0 || BD_k_i_<0 || BD_k_d_<0 || BD_deadzone_<0 || BD_windup_limit_ <0){
      max_velocity_ = 0.3;max_turn_rate_=4;line_row_location_=230;LF_k_p_=0.32;LF_k_i_=0.002;LF_k_d_=0.09;LF_windup_limit_=0.8;target_distance_=15,BD_k_p_=0.03;BD_k_i_=0.00016;BD_k_d_=0.0013;BD_windup_limit_=1.5;BD_deadzone_=0.03;
    dumpConfigValues();
    }


}
void dumpConfigValues()//dump the current parameters vlaues into dump.yaml so next time the robot will be launched, it will start with the same parameters from the last run
{
   std:: fstream OutFile; //The file which stores configuration parameters
   OutFile.open(DumpPath.c_str(), std::ifstream::out);
   if(OutFile.is_open())
	{	
		OutFile<<"max_velocity: "<<max_velocity_<<"\n";
		OutFile<<"max_turn_rate: "<<max_turn_rate_<<"\n";
		OutFile<<"line_row_location: "<<line_row_location_<<"\n";
		OutFile<<"LF_K_p: "<<LF_k_p_<<"\n";
		OutFile<<"LF_K_i: "<<LF_k_i_<<"\n";
		OutFile<<"LF_K_d: "<<LF_k_d_<<"\n";
		OutFile<<"LF_windup_limit: "<<LF_windup_limit_<<"\n";
		OutFile<<"BD_K_p: "<<BD_k_p_<<"\n";
		OutFile<<"BD_K_i: "<<BD_k_i_<<"\n";
		OutFile<<"BD_K_d: "<<BD_k_d_<<"\n";
		OutFile<<"BD_deadzone: "<<BD_deadzone_<<"\n";
		OutFile<<"BD_windup_limit: "<<BD_windup_limit_<<"\n";
		OutFile<<"target_distance: "<<target_distance_<<"\n";

	}



}

  /**
   * a parser for yaml file specified with 'FilePath'  to search for the parameter specified with 'parameter' string that return the value of the 'parameter' passed to the function
   * 
   *
   *
   * \param[in] parameter: the name of the parameter you want to extract from the dump.yaml file and,
   *  \FilePath: is the path for dump file you want to read the values from.
   * 
   */

std::string extractNumber(std:: string parameter,std:: string FilePath)
{
   std:: fstream inFile; //The file which stores configuration parameters

    double ParameterValue; // the extracted value from the file
   std:: string temp; // temporary string which will store the extracted intger value 
    size_t found; // location of the integer value within the line
   std:: string line;//each line read from the file
   inFile.open(FilePath.c_str(),std::ifstream::in);
    
    if (!inFile) {
        ROS_INFO("Unable to open file. File might have been deleted\n");

        return "-1"; // return 0.1 on error
    }
    
        while ( std::getline(inFile,line)) {
            
                      found = line.find(parameter);
                      if(found!= std::string::npos)// if the parameter is found within the line
                        {

    
                             for (unsigned int i=0; i < line.size(); i++) //check each character if it is a digit store it in temp
                                  {
       
                                      if (isdigit(line[i]))//
                                                           {
                                                            for (unsigned int a=i; a<line.size(); a++)
                                                              {
                                                                temp += line[a];               
                                                              }
                                                              //the first numeric block is extracted
                                                               break;
                                                            }
        
                                  }
                          

                           inFile.close();


                          return temp;
                        }
                                              }

   ROS_INFO("ParameterValue was not found\n");
    return "0";//return -1 if not found
}

  ReflekteStateMachine stateMachine;  double max_velocity_;    //!< The maximum velocity in m/s.
  double max_turn_rate_;   //!< The maximum turn-rate in rad/s.
  double target_distance_; //!< The target distance in m for following the blob.
  double LF_k_p_;          //!< Proportional component for the line-folling PID-controller.
  double LF_k_i_;          //!< Integral component for the line-folling PID-controller.
  double LF_k_d_;          //!< Derivative component for the line-folling PID-controller.
  double LF_windup_limit_; //!< Limits the absolute value of the integral term of the line-following PID-controller.
  double BD_k_p_;          //!< Proportional component for the blob-distance PID-controller.
  double BD_k_i_;          //!< Integral component for the blob-distance PID-controller.
  double BD_k_d_;          //!< Derivative component for the blob-distance PID-controller.
  double BD_windup_limit_; //!< Limits the absolute value of the integral term of the blob-distance PID-controller.
  double BD_deadzone_;     //!< Deadzone used for keeping the blob distance to avoid stuttering when robots not driving.
  int line_row_location_;       //!< Location where to look for the line that shoud be followed.
  std::vector<Marker> markers;  //!< All configured markers that should be considered.
  ros::Time last_command_time_; //!< Timestamp of the last published command.
  int tag_id_to_follow;    // ID of the AprilTag this robot should follow. TODO: make configurable, let higher level logic decide given the published AprilTagDetectionArray (e.g. pick closest)
  bool tag_to_follow_detected;
  apriltags2_ros::AprilTagDetection tag_to_follow; // Most recent detection of the AprilTag with ID tag_id_to_follow.
  
  ros::Subscriber feature_subscriber_;           //!< Subscriber to Features messages.
  ros::Subscriber tag_subscriber_;               //!< Subscriber to AprilTag detections.
  ros::Subscriber remote_control_subscriber_;    //!< Subscriber to RC command messages.
  ros::Subscriber dir_subscriber_;
  ros::Publisher cmd_publisher_;                 //!< Publisher for velocity commands.
  ros::Publisher state_publisher_;               //!< Publisher for the current state.
  ros::ServiceServer change_state_service_;      //!< Service handle for changing the state.
  ros::Publisher crossing_publisher_;            //!< Publisher for crossing.
  std::unique_ptr<ReconfigureServer> //!< Unique pointer to the reconfigurable parameter server for changing parameters.
  reconfigure_server_; //!< Unique pointer to the reconfigurable parameter server for changing parameters.
  PIDController line_following_pid_; //!< PIDController instance used to do line following.
  PIDController blob_distance_pid_;  //!< PIDController instance used to do keep blob distance.
  std::string DumpPath; //!< Path to dump file passed from the launch file.
  bool Initially_configed; //!< determiner to decide whether to load the parameters from dump file or the user input from rqt or webpage.
  bool isLeader = true;
  bool isIdle = false;

  std::string turnDirection;

  double previousVelocity = 1.0;

  bool isTurning = false;
  bool lostVlines = false;
};
}  // namespace cooperative_driving

#include "cooperative_driving/disable_ros_warnings_pre.h"
PLUGINLIB_EXPORT_CLASS(cooperative_driving::ReflekteNodelet, nodelet::Nodelet)
#include "cooperative_driving/disable_ros_warnings_post.h"
