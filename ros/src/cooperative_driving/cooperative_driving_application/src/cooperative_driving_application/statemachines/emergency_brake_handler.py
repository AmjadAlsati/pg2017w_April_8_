#!/usr/bin/env python

import rospy
import smach
import smach_ros
from smach_ros import ServiceState

from _common_containers import PublishMessage, WaitInterval
from cooperative_driving_networking.msg import EmergencyBrake
from cooperative_driving_logic.srv import ChangeState
from cooperative_driving_logic.srv import ChangeStateRequest as behaviors


def handle_emergency_brake_internal(ud, message):
    """!
    Handles an emergency brake message which was received internally from the robot due to an obstacles recognized by the distance sensors.

    @param message: An emergency brake message received from the robot
    """
    if isinstance(message, EmergencyBrake):
        if message.enable is True:
            # we just did an emergency brake
            rospy.logdebug("Received an internal emergency brake message due to a brake situation")
        else:
            # the local situation is cleared, let's go again
            rospy.logdebug("Received an internal emergency brake message due to a resolved brake situation")

        ud.message = message
        return True
    else:
        return False

@smach.cb_interface(input_keys=['message'], output_keys=['enable'])
def change_driving_behavior_cb_head(userdata, msg):
    #state = ChangeState()
    if userdata.message.enable is True:
        state = behaviors.IDLE
    else:
        #TODO assign the initial behaviour as in the original implementation, hardcoding it here
        state = behaviors.FOLLOW_LINE
    
    userdata.enable = userdata.message.enable

    return state


@smach.cb_interface(input_keys=['message'], output_keys=['enable'])
def change_driving_behavior_cb_member(userdata):
    #state = ChangeState()
    if userdata.message.enable is True:
        state = behaviors.IDLE
    else:
        #TODO assign the initial behaviour as in the original implementation, hardcoding it here
        state = behaviors.FOLLOW_BLOB
    
    userdata.enable = userdata.message.enable

    return state

def handle_emergency_brake(ud, message):
    """!
    Handles an external emergency brake message.

    Currently, all robots just stop if they receive an emergency brake message and only continue driving if they receive a second message from the same sender.

    Later, the robot should brake individually:
    * It stores the ids of all senders of an emergency brake messages whenever the logic decides to brake
    * The id of a sender is removed from this list if a corresponding message with enable=False was received from the same sender
    * It continues driving only if all brake situations are resolved (all ids are removed from the list)


    @param sender_id: The id of the robot which sent the received emergency brake message
    @param enable: Flag to indicate the the brake state
    """

    rospy.loginfo("Received an external emergency brake message (" + str(message.sender_id) + ", " + str(message.enable) + ")")

    if message.enable is True:
        # let's brake whenever we receive an emergency brake message (for the lulz)
        if handle_emergency_brake.last_emergency_brake_sender_id is None:
            handle_emergency_brake.last_emergency_brake_sender_id = message.sender_id

            # change the driving state to emergency brake (i.e. idle)
            retval = True
        else:
            retval = False
    else:
        if handle_emergency_brake.last_emergency_brake_sender_id == message.sender_id:
            # brake situation for this robot is cleared
            handle_emergency_brake.last_emergency_brake_sender_id = None

            retval = True
        else:
            retval = False

    ud.message = message
    return retval

def emergency_brake_internal_sm_head():
    sm = smach.StateMachine(outcomes=['preempted', 'aborted','failed', 'TBD'], input_keys=['robot_id'])
    with sm:
        smach.StateMachine.add('RCV_INTERNAL_EB',smach_ros.MonitorState('/robot/emergency_brake',
                               EmergencyBrake,
                               handle_emergency_brake_internal,
                               max_checks=1, output_keys=['message']),
                               transitions={'invalid':'failed', 'valid':'CHANGE_STATE'},
                               )
        smach.StateMachine.add('CHANGE_STATE',
                                ServiceState('/reflekte/change_state',
                                ChangeState,
                                request_cb = change_driving_behavior_cb_head,
                                input_keys = ['message']),
                                transitions={'succeeded':'PUB_MESSAGE'}
                              )
        smach.StateMachine.add('PUB_MESSAGE', PublishMessage('EMERGENCY_BRAKE'),
                                transitions={'done':'RCV_INTERNAL_EB'}
                               )
    return sm

def emergency_brake_internal_sm_member():
    sm = smach.StateMachine(outcomes=['preempted', 'aborted','failed', 'TBD'], input_keys=['robot_id'])
    with sm:
        smach.StateMachine.add('RCV_INTERNAL_EB',smach_ros.MonitorState('/robot/emergency_brake',
                               EmergencyBrake,
                               handle_emergency_brake_internal,
                               max_checks=1, output_keys=['message']),
                               transitions={'invalid':'failed', 'valid':'CHANGE_STATE'},
                               )
        smach.StateMachine.add('CHANGE_STATE',
                                ServiceState('/reflekte/change_state',
                                ChangeState,
                                request_cb = change_driving_behavior_cb_member,
                                input_keys = ['message']),
                                transitions={'succeeded':'RCV_INTERNAL_EB'}
                              )

    return sm

def emergency_brake_external_sm_head():
    sm = smach.StateMachine(outcomes=['preempted', 'aborted','failed', 'TBD'], input_keys=['robot_id'])
    handle_emergency_brake.last_emergency_brake_sender_id = None
    with sm:
        smach.StateMachine.add('RCV_EMERGENCY_BRAKE',
                                smach_ros.MonitorState('/networking/emergency_brake',
                                EmergencyBrake, handle_emergency_brake, max_checks=1, output_keys=['message']),
                                transitions={'invalid':'RCV_EMERGENCY_BRAKE', 'valid':'CHANGE_STATE'}
                                )
        smach.StateMachine.add('CHANGE_STATE',
                                ServiceState('/reflekte/change_state',
                                ChangeState,
                                request_cb = change_driving_behavior_cb_head,
                                input_keys = ['message']),
                                transitions={'succeeded':'RCV_EMERGENCY_BRAKE'}
                              )
    return sm

def emergency_brake_external_sm_member():
    sm = smach.StateMachine(outcomes=['preempted', 'aborted','failed', 'TBD'], input_keys=['robot_id'])
    with sm:
        smach.StateMachine.add('RCV_EMERGENCY_BRAKE',
                                smach_ros.MonitorState('/networking/emergency_brake',
                                EmergencyBrake, handle_emergency_brake, max_checks=1, output_keys=['message']),
                                transitions={'invalid':'RCV_EMERGENCY_BRAKE', 'valid':'CHANGE_STATE'},
                            )
        smach.StateMachine.add('CHANGE_STATE',
                                ServiceState('/reflekte/change_state',
                                ChangeState,
                                request_cb = change_driving_behavior_cb_member,
                                input_keys = ['message']),
                                transitions={'succeeded':'RCV_EMERGENCY_BRAKE'}
                              )
    return sm
