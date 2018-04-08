#!/usr/bin/env python

import rospy
import smach
import threading
from rospy import Publisher, Subscriber

from cooperative_driving_application import InternalController
from cooperative_driving_msgs.msg import Command
from cooperative_driving_networking.msg import CooperativeAwarenessMessage as Cam
from cooperative_driving_networking.msg import EmergencyBrake, Token, Platooning
from cooperative_driving_msgs.msg import Command
from cooperative_driving_application import create_cooperative_awareness_message
from cooperative_driving_application import create_command_message
from cooperative_driving_application import create_token_message, create_emergency_braking_message
from cooperative_driving_hardware.msg import LedCommand
from cooperative_driving_hardware.msg import LedValue



class FlashLED(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','preempted', 'failed'], input_keys=['flash_pattern'])
        self._led_pub = rospy.Publisher('/led_controller/cmd_led', LedCommand,
                                        queue_size=10)
        ## The name of the first LED (color)
        self._led1_name = 'led/red'
        ## The name of the second LED (color)
        self._led2_name = 'led/blue'

        #TODO: Move it to a more appropriate place
        ## In seconds, the time interval for flashing the LEDs
        self.LED_DURATION = 3.0
        ## In seconds, the time to sleep between two LED flashes
        self.LED_SEQUENCE_SLEEP = 0.2
        ## In seconds, the time the LEDs are on in the sequence
        self.LED_SEQUENCE_LED_DURATION = 0.2
    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt
            return 'preempted'
        
        # FIXME Do we want to keep the InternalController or wrap all functionality in states' execute() methods?
        #int_ctrl_instance = InternalController(ud.robot_id)
        # Flash LEDs once
        #int_ctrl_instance.flash_leds()
        if ud.flash_pattern == 'flash':
            self.publish_led_command(1, 1)
            rospy.sleep(rospy.Duration(self.LED_DURATION))
            self.publish_led_command(0, 0)
        elif ud.flash_pattern == 'off':
            self.publish_led_command(0, 0)
        elif ud.flash_pattern == 'sequence':
            for _ in range(int(self.LED_DURATION / (self.LED_SEQUENCE_SLEEP + self.LED_SEQUENCE_LED_DURATION))):
                self.publish_led_command(1,1)
                rospy.sleep(self.LED_SEQUENCE_LED_DURATION)

                self.publish_led_command(0,0)
                rospy.sleep(self.LED_SEQUENCE_SLEEP)

                if self.preempt_requested():
                    self.service_preempt
                    return 'preempted'

        return 'done'
    def publish_led_command(self, led1_value, led2_value):
        """!
        Constructs and publishes LedCommand message
        """
        led_cmd = LedCommand()
        led_cmd.header.stamp = rospy.Time.now()
        led_1_value = LedValue()
        led_2_value = LedValue()
        led_1_value.value = led1_value
        led_2_value.value = led2_value
        led_1_value.frame_id = self._led1_name
        led_2_value.frame_id = self._led2_name
        led_cmd.values = [led_1_value, led_2_value]
        
        rospy.logdebug("Publishing LedCommand message: " + str(led_cmd))
        self._led_pub.publish(led_cmd)

class TimeoutState(smach.State):

    OUTCOME_RECEIVED = 'received'
    OUTCOME_TIMEOUT = 'timeout'
    OUTCOME_PREEMPTED = 'preempted'

    """
    A state that will wait for a message on a ROS topic or until a certain time has passed.
    """
    def __init__(self, topic, msg_type, timeout, count=1, input_keys=[], output_keys=['message']):
        """State constructor
        @type topic string
        @param topic the topic to monitor

        @type msg_type a ROS message type
        @param msg_type determines the type of the monitored topic

        @type timeout rospy.Duration
        @param timeout the maximum time to wait for a message. In order to omit the timeout, pass None.

        @type count int
        @param count the number of messages to wait for.
        """
        smach.State.__init__(
            self,
            outcomes=[self.__class__.OUTCOME_RECEIVED, self.__class__.OUTCOME_TIMEOUT,
                      self.__class__.OUTCOME_PREEMPTED],
            input_keys=input_keys,
            output_keys=output_keys)
        self._topic = topic
        self._msg_type = msg_type
        self._timeout = timeout
        self._expected_message_count = count
        self._trigger_event = threading.Event()

    def execute(self, ud):
        # If prempted before even getting a chance, give up.
        if self.preempt_requested():
            self.service_preempt()
            return self.__class__.OUTCOME_PREEMPTED

        self._received_message_count = 0
        self._trigger_event.clear()

        self._sub = rospy.Subscriber(self._topic, self._msg_type, self._message_cb, callback_args=ud)
        # might want to add a timeout callback for additional functionality
        if self._timeout:
            self._timeout_timer = rospy.Timer(self._timeout, self._timer_cb, oneshot=True)

        self._trigger_event.wait()
        self._sub.unregister()
        if self._timeout:
            self._timeout_timer.shutdown()

        if self.preempt_requested():
            self.service_preempt()
            return self.__class__.OUTCOME_PREEMPTED

        if self._received_message_count == self._expected_message_count:
            return self.__class__.OUTCOME_RECEIVED

        return self.__class__.OUTCOME_TIMEOUT

    def _message_cb(self, message, ud):
        # Save received message in output userdata for processing in follow-up states
        ud.message = message

        self._received_message_count += 1

        if self._received_message_count == self._expected_message_count:
            self._trigger_event.set()
    
    def _timer_cb(self, event):
        self._trigger_event.set()

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.set()

class PublishMessage(smach.State):
    def __init__(self, type):
        smach.State.__init__(self, outcomes=['done', 'failed', 'preempted'],
        input_keys=['robot_id', 'velocity', 'steering', 'token_id', 'enable'])

        if (type == 'CAM'):
            self._pub = Publisher('networking/cam', Cam, queue_size=10)
            self.type = type
        elif (type == 'TOKEN'):
            self._pub = Publisher('/networking/token', Token, queue_size=10)
            self.type = type
        elif (type == 'EMERGENCY_BRAKE'):
            self._pub = Publisher('/networking/emergency_brake', EmergencyBrake, queue_size=10)
            self.type = type
        elif (type == 'PLATOONING'):
            self._pub = Publisher('/networking/platooning', Platooning, queue_size=10)
            self.type = type
        elif (type == 'COMMAND'):
            self._pub = Publisher('/networking/command', Command, queue_size=10)
            self.type = type
        else:
            rospy.logerr('Unknown message type')

    def execute(self, ud):
        #global robot_id, velocity, steering
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if (self.type == 'CAM'):
            msg = self.create_cam(ud.robot_id, ud.velocity, ud.steering)
        elif (self.type == 'TOKEN'):
            msg = self.create_token(ud.robot_id, ud.token_id)
        elif (self.type == 'EMERGENCY_BRAKE'):
            msg = self.create_eb_msg(ud.robot_id, ud.enable)
        elif (self.type == 'PLATOONING'):
            msg = self.create_platooning_msg(ud.robot_id)
        elif (self.type == 'COMMAND'):
            msg = self.create_command(ud.command, ud.params)
        else:
            return 'failed'
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        #TODO check publish status
        rospy.sleep(2.)
        rospy.logdebug("Transmitting network message: " + str(msg))
        self._pub.publish(msg)
        rospy.logdebug("Transmitted message")

        return 'done'

    def create_cam(self, robot_id, velocity, steering):
        cam = create_cooperative_awareness_message(robot_id, velocity, steering)
        return cam
    def create_token(self, robot_id, token_id):
        tokenMsg = create_token_message(robot_id, token_id)
        return tokenMsg
    def create_platooning_msg(self, robot_id):
        pass
    def create_eb_msg(self, robot_id, enable):
        ebMsg = create_emergency_braking_message(robot_id, enable)
        return ebMsg
    def create_command(self, command, params):
        pass

class WaitInterval(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','preempted', 'failed'],
        input_keys=['interval'])

    def execute(self, ud):
        #TODO Add preemption routine
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        rospy.sleep(ud.interval)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        return 'done'