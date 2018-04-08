#!/usr/bin/env python

import rospy
import smach
import smach_ros

class Start_HW(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed', 'preempted'])

    def execute(self, userdata):
        if self.preempt_requested():
             self.service_preempt()
             return 'preempted'

        rospy.sleep(2.)
        rospy.loginfo('Initializing hardware and other components needed')

        return 'success'

class Select_mode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['head_node','member_node', 'preempted'],
                                   input_keys=['robot_id'])


    def execute(self, ud):
        if self.preempt_requested():
             self.service_preempt()
             return 'preempted'
 
        rospy.sleep(2.)
        rospy.loginfo('Selecting the robot state')

        # Use the robot id to decide the head now
        # Later specific algorithms may be defined to identify the same
        if (ud.robot_id > 0):
            return 'member_node'
        else:
            return 'head_node'

def init_robot_sm():
    sm = smach.StateMachine(outcomes=['head_node','member_node', 'failed', 'preempted'], input_keys=['robot_id'])
    with sm:
        smach.StateMachine.add('START_HW', Start_HW(), transitions={'success':'SELECT_MODE'})
        smach.StateMachine.add('SELECT_MODE', Select_mode())
    return sm