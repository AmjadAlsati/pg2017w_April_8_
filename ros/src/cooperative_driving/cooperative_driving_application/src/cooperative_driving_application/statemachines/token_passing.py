#!/usr/bin/env python

import rospy
import smach
import smach_ros
from _common_containers import FlashLED, PublishMessage, TimeoutState
from cooperative_driving_networking.msg import Token

## Constants
# In seconds, time to wait before considering a token to be lost
# TODO: Collect constants in a central file?
TOKEN_TIMEOUT = 6

class TokenPassing(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['done','preempted', 'failed'], input_keys=['robot_id'])

class CreateToken(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','preempted', 'failed'], io_keys=['token_id'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt
            return 'preempted'

        # Create new token
        ud.token_id += 1
        rospy.loginfo("Created token with token_id" + str(ud.token_id))

        return 'done'

def check_token(ud, msg):
    # Check whether Token message was sent by predecessor
    rospy.loginfo("sender_id: " + str(msg.sender_id) + ", own robot_id: " + str(ud.robot_id))
    return msg.sender_id + 1 == ud.robot_id

def token_sm_head():
    # Create top level state machine
    sm = TokenPassing()
    sm.userdata.token_id = 0
    sm.userdata.flash_pattern = 'flash'

    with sm:
        smach.StateMachine.add('CREATE_TOKEN', CreateToken(), {'done':'FLASH_LED'})
        smach.StateMachine.add('FLASH_LED', FlashLED(), {'done':'SEND_TOKEN'})
        smach.StateMachine.add('SEND_TOKEN', PublishMessage('TOKEN'), {'done':'WAIT_FOR_TOKEN'})
        smach.StateMachine.add('WAIT_FOR_TOKEN', TimeoutState('/networking/token', Token, rospy.Duration(TOKEN_TIMEOUT)), {'timeout':'FLASH_LED', 'received':'WAIT_FOR_TOKEN'})
    
    # Return final statemachine
    return sm

def token_sm_member():
    # Create top level state machine
    sm = TokenPassing()
    sm.userdata.flash_pattern = 'flash'

    with sm:
        smach.StateMachine.add('WAIT_FOR_TOKEN', smach_ros.MonitorState('/networking/token', Token, check_token, max_checks=1, input_keys=['robot_id']), {'valid':'FLASH_LED', 'invalid':'WAIT_FOR_TOKEN'})
        smach.StateMachine.add('FLASH_LED', FlashLED(), {'done':'SEND_TOKEN'})
        smach.StateMachine.add('SEND_TOKEN', PublishMessage('TOKEN'), {'done':'WAIT_FOR_TOKEN'})

    # Return final statemachine
    return sm