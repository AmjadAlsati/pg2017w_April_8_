#!/usr/bin/env python

import rospy
import smach
import smach_ros

from _common_containers import PublishMessage, WaitInterval
from cooperative_driving_networking.msg import CooperativeAwarenessMessage as Cam

def rcv_cam(ud, message):
    """!
    Callback for handling reception of a CAM

    @param message: A received CAM which data has to be handled
    """

    rospy.loginfo("Received CAM from " + str(message.sender_id))
    ud.msg = message
    return True

class ProcessCam(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','preempted', 'failed'],
        input_keys=['msg'])

    def execute(self, ud):
        if self.preempt_requested():
             self.service_preempt()
             return 'preempted'

        rospy.sleep(2.)
        # TODO: Process Cam here
        return 'done'

def handle_beaconing_sm():
    sm = smach.StateMachine(outcomes=['preempted', 'aborted','failed', 'TBD'], input_keys=['robot_id', 'velocity', 'steering', 'interval'])
    with sm:
        smach.StateMachine.add('WAIT_CAM', WaitInterval(), transitions={'done':'PUB_CAM'})
        smach.StateMachine.add('PUB_CAM', PublishMessage('CAM'), transitions={'done':'WAIT_CAM'})
    return sm

def handle_cam_sm():
    sm = smach.StateMachine(outcomes=['preempted', 'aborted','failed', 'TBD'])
    with sm:
        smach.StateMachine.add('RCV_CAM', smach_ros.MonitorState("/networking/cam", Cam, rcv_cam, max_checks=1, output_keys=['msg']), transitions={'invalid':'failed', 'valid':'PROCESS_CAM'})
        smach.StateMachine.add('PROCESS_CAM', ProcessCam(), transitions={'done':'RCV_CAM'})
    
    return sm