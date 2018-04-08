#!/usr/bin/env python

import rospy
import smach
import smach_ros
from smach import CBState
from smach_ros import ServiceState

from actionlib import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from cooperative_driving_logic.srv import ChangeState
from cooperative_driving_logic.srv import ChangeStateRequest as behaviors


from cooperative_driving_logic.srv import ChangeState, ChangeStateRequest


# Import nested smach containers
from cooperative_driving_application.statemachines import token_sm_head, token_sm_member, handle_beaconing_sm, handle_cam_sm, init_robot_sm
from cooperative_driving_application.statemachines import emergency_brake_external_sm_head, emergency_brake_external_sm_member, emergency_brake_internal_sm_head, emergency_brake_internal_sm_member

cmd_pub = None
change_reflekte_state = None


class InitHead(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['auto_mode','rc_mode', 'failed', 'preempted'],
        input_keys=['rc'])
        

    def execute(self, ud):
        if self.preempt_requested():
             self.service_preempt()
             return 'preempted'
        rospy.sleep(2.)
        rospy.loginfo('Initializing Head node')

        if ud.rc == 'true':
            return 'rc_mode'
        else:
            return 'auto_mode'

class InitMember(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['auto_mode','rc_mode', 'failed', 'preempted'],
        input_keys=['rc'])

    def execute(self, ud):
        if self.preempt_requested():
             self.service_preempt()
             return 'preempted'

        rospy.sleep(2.)
        rospy.loginfo('Initializing member node')

        if ud.rc == 'true':
            return 'rc_mode'
        else:
            return 'auto_mode'

def joy_control_callback(ud, msg):
    """ Callback from joy_control publisher.

    @param msg Message that arrived on the joy topic.
    """
    button_map = {
        0: ChangeStateRequest.IDLE,
        1: ChangeStateRequest.FOLLOW_LINE,
        2: ChangeStateRequest.PLATOONING,
        3: ChangeStateRequest.FOLLOW_BLOB,
        4: ChangeStateRequest.DYNAMIC_FOLLOW_LINE,
        5: ChangeStateRequest.REMOTE_CONTROL
    }

    twist_msg = Twist()
    twist_msg.linear.x = msg.axes[1]
    twist_msg.angular.z = -1 * msg.axes[2]
    cmd_pub.publish(twist_msg)
    if 1 in msg.buttons and msg.buttons.index(1) in button_map:
        change_reflekte_state(button_map[msg.buttons.index(1)])
    return True

def exit_cb(ud, msg):
    print "Stopping the robot"
    return False

def store_latest_velocity(ud, message):
    """!
    Callback for storing the latest velocity

    @param message: A cmd_vel message which in contins the latest velocity and steering values
    """

    ud.velocity = message.linear.x
    ud.steering = message.angular.z
    return True

def child_term_cb(outcome_map):
    if outcome_map['HANDLE_BEACONING'] == 'failed':
        return True
    elif outcome_map['EXIT'] == 'invalid':
        return True
    else:
        return False

def out_cb(outcome_map):
    if outcome_map['EXIT'] == 'invalid':
        return 'exit'
    elif outcome_map['HANDLE_BEACONING'] == 'failed':
        return 'error'
    else:
        return 'restart'

def createStatemachine(robot_id, rc):
    
    # Initialize top statmachine container
    smRobot = smach.StateMachine(outcomes=['exit_on_error','preempted', 'aborted','exit_on_request', 'TBD'])
    smRobot.userdata.robot_id = robot_id
    smRobot.userdata.rc = rc

    
    #smInitRobot.userdata.robot_id = robot_id
    with smRobot:
        
        smach.StateMachine.add('INIT_ROBOT', init_robot_sm(), transitions={'head_node':'ROBOT_HEAD','member_node':'ROBOT_MEMBER', 'failed':'exit_on_error', 'preempted':'preempted'})
        smRobotHead = smach.StateMachine(outcomes=['preempted', 'aborted','failed', 'exit', 'TBD'], input_keys=['rc', 'robot_id'])
        with smRobotHead:
            smach.StateMachine.add('INIT_HEAD', InitHead(), transitions={'auto_mode':'SET_STATE','rc_mode':'HANDLE_RC'})
            smach.StateMachine.add('HANDLE_RC', smach_ros.MonitorState('/joy', Joy, joy_control_callback, max_checks=1), transitions={'valid':'HANDLE_RC', 'invalid':'failed'})
            smach.StateMachine.add('SET_STATE',
                                ServiceState('/reflekte/change_state',
                                ChangeState,
                                request = ChangeState._request_class(behaviors.FOLLOW_LINE, 0)),
                                transitions={'succeeded':'HANDLE_AUTO'}
                              )
        
            # Auto mode
            conAutoModehead = smach.Concurrence(
                outcomes=['exit', 'preempted', 'error', 'aborted'],
                default_outcome='exit',
                child_termination_cb=child_term_cb,
                outcome_cb=out_cb,
                input_keys =['robot_id']
                )

            conAutoModehead.userdata.interval = 10.0
            conAutoModehead.userdata.velocity = 0.0
            conAutoModehead.userdata.steering = 0.0

            with conAutoModehead:
                smach.Concurrence.add('TOKEN_PASSING', token_sm_head())
                smach.Concurrence.add('EXIT', smach_ros.MonitorState("/exit", Empty, exit_cb))
                smach.Concurrence.add('UPDATE_DATA', smach_ros.MonitorState('/drive_controller/cmd_vel', Twist, store_latest_velocity, output_keys=['velocity', 'steering']))
                smach.Concurrence.add('HANDLE_BEACONING', handle_beaconing_sm())
                smach.Concurrence.add('HANDLE_EXT_CAM', handle_cam_sm())
                smach.Concurrence.add('HANDLE_EMERGENCY_BRAKE_INT', emergency_brake_internal_sm_head())
                smach.Concurrence.add('HANDLE_EMERGENCY_BRAKE_EXT', emergency_brake_external_sm_head())
                               #smach.Concurrence.add('HANDLE_TURN', )
            smach.StateMachine.add('HANDLE_AUTO', conAutoModehead, transitions={'error':'failed', 'aborted':'failed'})
        smach.StateMachine.add('ROBOT_HEAD', smRobotHead, transitions={'failed':'exit_on_error', 'exit':'exit_on_request'})

        smRobotMember = smach.StateMachine(outcomes=['preempted', 'aborted','failed', 'exit', 'TBD'], input_keys=['rc', 'robot_id'])
        with smRobotMember:
            smach.StateMachine.add('INIT_MEMBER', InitMember(), transitions={'auto_mode':'SET_STATE','rc_mode':'HANDLE_RC'})
            smach.StateMachine.add('HANDLE_RC', smach_ros.MonitorState('/joy', Joy, joy_control_callback, max_checks=1), transitions={'valid':'HANDLE_RC', 'invalid':'failed'})
            smach.StateMachine.add('SET_STATE',
                                    ServiceState('/reflekte/change_state',
                                    ChangeState,
                                    request = ChangeState._request_class(behaviors.FOLLOW_BLOB, robot_id-1)),
                                    transitions={'succeeded':'HANDLE_AUTO'}
                                    )
            # Auto mode
            conAutoModeMember = smach.Concurrence(
                outcomes=['exit', 'preempted', 'error', 'aborted'],
                default_outcome='exit',
                child_termination_cb=child_term_cb,
                outcome_cb=out_cb,
                input_keys =['robot_id']
                )

            conAutoModeMember.userdata.interval = 10.0
            conAutoModeMember.userdata.velocity = 0.0
            conAutoModeMember.userdata.steering = 0.0

            with conAutoModeMember:
                smach.Concurrence.add('TOKEN_PASSING', token_sm_member())
                smach.Concurrence.add('EXIT', smach_ros.MonitorState("/exit", Empty, exit_cb))
                smach.Concurrence.add('UPDATE_DATA', smach_ros.MonitorState('/drive_controller/cmd_vel', Twist, store_latest_velocity, output_keys=['velocity', 'steering']))
                smach.Concurrence.add('HANDLE_BEACONING', handle_beaconing_sm())
                smach.Concurrence.add('HANDLE_EXT_CAM', handle_cam_sm())
                smach.Concurrence.add('HANDLE_EMERGENCY_BRAKE_INT', emergency_brake_internal_sm_member())
                smach.Concurrence.add('HANDLE_EMERGENCY_BRAKE_EXT', emergency_brake_external_sm_member())
                #smach.Concurrence.add('HANDLE_TURN', )

            smach.StateMachine.add('HANDLE_AUTO', conAutoModeMember, transitions={'error':'failed'})
        
        smach.StateMachine.add('ROBOT_MEMBER', smRobotMember, transitions={'failed':'exit_on_error', 'exit':'exit_on_request'})

    return smRobot


def main():
    
    global cmd_pub, change_reflekte_state

    rospy.init_node('Test')

    robot_id = rospy.get_param('~id')
    rc = rospy.get_param('~rc')
    #robot_id = 0, remapping={'rc':'rc', 'robot_id':'robot_id'}
    #rc = 'false'

    if (rc == "True"):
        print"REMOTE CONTROL ACTIVE"
        cmd_pub = rospy.Publisher('/remote_control/cmd_vel', Twist, queue_size=1)
        change_reflekte_state = rospy.ServiceProxy('/reflekte/change_state', ChangeState)

    sm = createStatemachine(robot_id, rc)


    sis = smach_ros.IntrospectionServer('smach_introspection', sm, '/ROBOT')
    sis.start()
    #rospy.sleep(10.)
    outcome = sm.execute()

    rospy.spin()


if __name__ == '__main__':
    main()
