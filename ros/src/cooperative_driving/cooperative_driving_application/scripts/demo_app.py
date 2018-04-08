#!/usr/bin/env python

"""!@package cooperative_driving_application
Defines an application used for demonstration purposes in the pg2016w.
"""

import abc

import rospy

from cooperative_driving_application import (
    InternalController,
    ExternalController,
    ONE_RTT
)
from cooperative_driving_common import Pubscriber
from cooperative_driving_application import create_token_message
from cooperative_driving_logic.srv import ChangeStateRequest as behaviors
from cooperative_driving_networking.msg import EmergencyBrake, Token
from dynamic_reconfigure.server import Server
from cooperative_driving_application.cfg import applicationServerConfig
from cooperative_driving_msgs.msg import Command


# constants

## Number of robots in demo, used for reducing the maximum time to wait before a token is considered as lost
NUMBER_OF_ROBOTS = 4
## In seconds, time to wait before considering the token to be lost
TOKEN_TIMEOUT_DURATION = ONE_RTT * NUMBER_OF_ROBOTS
## In seconds, the time to wait for an expected predecessor
PREDECESSOR_TIMEOUT_DURATION = ONE_RTT
## In seconds, the time to wait for an expected new tail
TAIL_TIMEOUT_DURATION = ONE_RTT
global BEACON_INTERVAL
BEACON_INTERVAL = 1

## A reference to the application instance
global app
app = None

def callback(config, level):
    global BEACON_INTERVAL
    print("""Beacon Interval: {beaconing_interval}""".format(**config))
    print("""Beacon config: {enable_beaconing}""".format(**config))
    BEACON_INTERVAL = config.get('beaconing_interval')
    enableBeacon = config.get('enable_beaconing')
    sendCommand = config.get('announce')
    print("Cam",BEACON_INTERVAL)
    print("Cam",enableBeacon)
    if (app != None):
        if (enableBeacon == True):
            app.enableBeaconing(BEACON_INTERVAL)
        else:
            app.disableBeaconing()

    if (sendCommand == True):
        msg = Command()
        msg.command = "Turn"
        msg.turn.direction = "LEFT"
        app.handle_command(0, msg)
    
    return config

class DemoApplication(ExternalController, InternalController):
    """!
    Abstracts common behavior of different types of demonstration robots (Leader & Follower)

    It provides functionality for forwarding a simple token and emergency brake handling.

    Currently, it is not intended to change the role during runtime.
    This requires a complete refactoring of the inheritance relation between the classes.
    """

    __metaclass__ = abc.ABCMeta

    def __init__(self, robot_id):
        """!
        Initialization of a DemoApplication

        @param robot_id: The id of this robot given as a parameter during initialization
        """
        super(DemoApplication, self).__init__(robot_id)
        InternalController.__init__(self, robot_id)


        srv = Server(applicationServerConfig, callback)

        ## Stores the desired driving behavior of this robot
        self._desired_driving_behavior = None

        ## The id of the latest received token
        self._last_token_id = 0
        ## The id of the last known leader
        self._leader_id = 0

        ## The id of the robot which sent the emergecy brake message due to a brake situation
        self._last_emergency_brake_sender_id = None

        self.register_emergency_brake_cb(self.handle_emergency_brake)
        print"**********DEMO APPLICATION**************"
        
    def forward_token(self, event=None):
        """!
        Broadcasts a token message via networking

        @param event: An event which is forwarded from the caller according to the ROS API
        """

        # fill message
        msg = create_token_message(self._robot_id, self._last_token_id)

        self.publish_message(msg)

    @abc.abstractmethod
    def handle_token(self, sender_id, token_id):
        """!
        Dummy method for handling a token which is overridden in the sub-class, since Leader and Follower handle tokens differently

        @param sender_id: The id of the robot which sent the received token message
        @param token_id: The id of the received token
        """

        pass

    def handle_emergency_brake(self, sender_id, enable=True):
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

        rospy.loginfo("Received an external emergency brake message (" + str(sender_id) + ", " + str(enable) + ")")

        if enable is True:
            # let's brake whenever we receive an emergency brake message (for the lulz)
            if self._last_emergency_brake_sender_id is None:
                self._last_emergency_brake_sender_id = sender_id

                # change the driving state to emergency brake (i.e. idle)
                self.change_driving_behavior(behaviors.IDLE)
            # else:
                # already received a emergency brake message
        else:
            if self._last_emergency_brake_sender_id == sender_id:
                # brake situation for this robot is cleared
                self._last_emergency_brake_sender_id = None

                self.change_driving_behavior(self._desired_driving_behavior)
            # else:
                # we only react to the first sender of an emergency brake message

    def handle_emergency_brake_internal(self, message):
        """!
        Handles an emergency brake message which was received internally from the robot due to an obstacles recognized by the distance sensors.

        @param message: An emergency brake message received from the robot
        """

        if message.enable is True:
            # we just did an emergency brake
            rospy.logdebug("Received an internal emergency brake message due to a brake situation")
            # change the driving state to emergency brake (i.e. idle)
            self.change_driving_behavior(behaviors.IDLE)
        else:
            # the local situation is cleared, let's go again
            rospy.logdebug("Reveived an internal emergency brake message due to a resolved brake situation")
            # switch to previous normal state
            self.change_driving_behavior(self._desired_driving_behavior)

        # only if leader
        if self._robot_id == 0:
            # publish a message via external networking
            self.publish_message(message)
            rospy.logdebug("Published an external emergency brake message")

class Leader(DemoApplication):
    """!
    Encapsulates leader specific additional functionality
    """

    def __init__(self, robot_id):
        """!
        Initialization of a Leader

        @param robot_id: The id of this robot given as a parameter during initialization
        """

        super(Leader, self).__init__(robot_id)

        ## The id of the latest known tail (in the Platoon)
        self._last_tail_id = 0
        ## Stores a timer for the duration in which we expect a packet from a expected new tail
        self._tail_timer = None

        ## Stores a timer for creating a new token it the old one gets lost
        self._token_timer = None

        ## Stores the desired driving behavior of this leading robot
        self._desired_driving_behavior = behaviors.FOLLOW_LINE

        self.change_driving_behavior(self._desired_driving_behavior)
        self.create_token()

        self.flash_leds_sequence()
        self.handle_led_timeout(self.forward_token)
        self.register_token_cb(self.handle_token)
        global BEACON_INTERVAL
        self.enableBeaconing(BEACON_INTERVAL)

    def create_token(self):
        """!
        Creates a new token and flashes the LEDs
        """

        self._last_token_id += 1
        rospy.loginfo("Created new token " + str(self._last_token_id))

    def forward_token(self, event=None):
        """!
        Forwards the token and starts a token timer.
        Overrides method from DemoApplication.

        @param event: An event which is forwarded from the caller according to the ROS API
        """

        DemoApplication.forward_token(self)

        # After sending, start receive timeout
        self._token_timer = rospy.Timer(rospy.Duration(TOKEN_TIMEOUT_DURATION), self.handle_token_timeout, True)

    def handle_token_timeout(self, event=None):
        """!
        Callback for token timout.
        Creates a new token.

        @param event: An event which is forwarded from the caller according to the ROS API
        """
        rospy.logdebug("Handling token timeout")

        self.create_token()

        self.flash_leds_sequence()
        self.handle_led_timeout(self.forward_token)

    def handle_tail_timeout(self, event=None):
        """!
        Callback for tail timout.
        Directly continues with token passing, thus creates a new token.

        @param event: An event which is forwarded from the caller according to the ROS API
        """
        rospy.logdebug("Handling tail timeout")

        self.create_token()

        self.flash_leds(self.forward_token)

    def handle_token(self, sender_id, token_id):
        """!
        Handles token from a received packet.
        Overrides method from DemoApplication.

        @param sender_id: The id of the robot which sent the received token message
        @param token_id: The id of the received token
        """

        if token_id == self._last_token_id:
            # valid token
            if sender_id > self._last_tail_id:
                # message from a robot futher at the end
                rospy.loginfo(
                    "Received valid token " +
                    str(token_id) +
                    " from " +
                    str(sender_id) +
                    " at the end of the platoon")

                self._last_tail_id = sender_id

                # abort tail timer, since there is a new tail now
                if self._tail_timer is not None:
                    self._tail_timer.shutdown()

                # Reset receive timeout
                if self._token_timer is not None:
                    self._token_timer.shutdown()

                self._token_timer = rospy.Timer(rospy.Duration(TOKEN_TIMEOUT_DURATION),
                                                self.handle_token_timeout, True)

            elif sender_id == self._last_tail_id:
                # message from last known tail
                rospy.loginfo("Received valid token " + str(token_id) + " from last known tail " + str(sender_id))

                # abort token timer, since we received the token from the last known tail
                if self._token_timer is not None:
                    self._token_timer.shutdown()

                # wait one timer more before directly starting new token round
                rospy.logdebug("Waiting for expected new tail...")

                if self._tail_timer is not None:
                    self._tail_timer.shutdown()

                self._tail_timer = rospy.Timer(rospy.Duration(TAIL_TIMEOUT_DURATION), self.handle_tail_timeout, True)

            rospy.loginfo("Waiting for more expected follower...")

        else:
            rospy.logdebug("Received packet with invalid token id or not from robot at the end of the platoon")

class Follower(DemoApplication):
    """!
    Encapsulates follower specific additional functionality
    """

    def __init__(self, robot_id):
        """!
        Initialization of a Follower

        @param robot_id: The id of this robot given as a parameter during initialization
        """

        super(Follower, self).__init__(robot_id)

        ## The id of the sender of the token which we are currently storing due to
        ## potential expected direct predecessor of this robot which this robot did
        ## not recognize yet
        self._expected_sender_id = 0
        ## The id of the token which we are currently storing due to potential
        ## expected direct predecessor of this robot which this robot did not
        ## recognize yet
        self._expected_token_id = 0
        ## Stores a timer for the duration in which we expect a packet from a expected predecessor
        self._predecessor_timer = None

        ## The id of the latest known predecessor
        self._predecessor_id = 0

        ## Stores the desired driving behavior of this following robot
        self._desired_driving_behavior = behaviors.FOLLOW_BLOB

        # TODO: For now, always follow the tag with ID=robot_id-1. This should probably change in the future.
        self.change_driving_behavior(self._desired_driving_behavior, tag_id_to_follow=robot_id-1)
        self.register_token_cb(self.handle_token)
        self.register_command_cb(self.handle_command)
        global BEACON_INTERVAL
        self.enableBeaconing(BEACON_INTERVAL)

    def accept_token(self, sender_id, token_id):
        """!
        Accepts a token and flashes the LEDs

        @param sender_id: The id of the robot which sent the correspondig message of the token which is to be accepted
        @param token_id: The id of the received token which is to be accepted
        """

        rospy.loginfo("Accepting token " + str(token_id) + " from " + str(sender_id))

        self._predecessor_id = sender_id
        self._last_token_id = token_id
        self.flash_leds(self.forward_token)

    def handle_predecessor_timeout(self, event=None):
        """!
        Callback for predecessor timeout.
        Assumes that there is no predecessor and accepts the token

        @param event: An event which is forwarded from the caller according to the ROS API
        """

        self.accept_token(self._expected_sender_id, self._expected_token_id)
        self._expected_sender_id = 0
        self._expected_token_id = 0

    def handle_token(self, sender_id, token_id):
        """!
        Handles token from a received packet.
        Overrides method from DemoApplication.

        @param sender_id: The id of the robot which sent the received token message
        @param token_id: The id of the received token
        """

        if sender_id == self._leader_id:
            # sender is leader
            if sender_id == (self._robot_id - 1):
                # the sender (leader) is direct predecessor
                # it is always correct --> accept token
                rospy.loginfo("Received valid token from leader (direct predecessor)")

                # Reset predecessor timeout
                if self._predecessor_timer is not None:
                    self._predecessor_timer.shutdown()

                self.accept_token(sender_id, token_id)
            elif sender_id < (self._robot_id - 1):
                # sender is not direct predecessor (in terms of id)
                if token_id > self._last_token_id:
                    # token id is higher
                    # valid token, but maybe not for us
                    # save token_id and wait for expected direct predecessor
                    rospy.loginfo("Storing token and waiting for expected predecessor")
                elif token_id == self._last_token_id:
                    # token id is the same
                    # error case
                    # could also be that the missed the restart
                    # since this is not the direct predecessor, wait to receive it again
                    rospy.logerr("Error case: Token id is the same (leader, not direct predecessor)")
                else:
                    # token_id < self._last_token_id
                    # reset message? maybe we missed the restart
                    # reset token_id but do not assume to have the token
                    rospy.loginfo("Storing token and waiting for expected predecessor")

                # wait for expected direct predecessor
                self._expected_sender_id = sender_id
                self._expected_token_id = token_id
                id_diff = self._robot_id - sender_id

                # Reset predecessor timeout
                if self._predecessor_timer is not None:
                    self._predecessor_timer.shutdown()

                self._predecessor_timer = rospy.Timer(rospy.Duration(PREDECESSOR_TIMEOUT_DURATION * id_diff),
                                                      self.handle_predecessor_timeout, True)
            # else:
                # sender_id > (self._robot_id - 1)
                # sender_id > self._robot_id
                # do not accept, since the message came from a successor
                # rospy.logdebug("Ignore case: Message from a successor")
        else:
            # sender is not leader
            if sender_id == (self._robot_id - 1):
                # sender is direct predecessor (in terms of id)
                if token_id > self._last_token_id:
                    # token id is higher
                    # default case
                    # accept token
                    rospy.loginfo("Received valid token from direct predecessor (not leader)")

                    # Reset predecessor timeout always, since message from direct predecessor
                    if self._predecessor_timer is not None:
                        self._predecessor_timer.shutdown()

                    self.accept_token(sender_id, token_id)
                elif token_id == self._last_token_id:
                    # token id is the same
                    # received the same token twice
                    # error case
                    rospy.logerr("Error case: Token id is the same (direct predecessor)")
                else:
                    # token_id < self._last_token_id
                    # new token round? Did we miss the restart? wait for receiving it from the leader
                    rospy.logwarn("Did we miss the restart? Resetting token id")
                    self._last_token_id = token_id

                # Reset predecessor timeout always, since message from direct predecessor
                if self._predecessor_timer is not None:
                    self._predecessor_timer.shutdown()

            elif sender_id < (self._robot_id - 1):
                # sender is not direct predecessor (in terms of id)
                if token_id > self._last_token_id:
                    # token id is higher
                    # valid token, but maybe not for us
                    # save token_id and wait for expected direct predecessor

                    # wait for expected direct predecessor
                    self._expected_sender_id = sender_id
                    self._expected_token_id = token_id
                    id_diff = self._robot_id - sender_id

                    # Reset predecessor timeout
                    if self._predecessor_timer is not None:
                        self._predecessor_timer.shutdown()

                    self._predecessor_timer = rospy.Timer(rospy.Duration(PREDECESSOR_TIMEOUT_DURATION * id_diff),
                                                          self.handle_predecessor_timeout, True)

                    rospy.loginfo("Storing token and waiting for expected predecessor")
                elif token_id == self._last_token_id:
                    # token id is the same
                    # received the same token twice
                    # error case
                    rospy.logerr("Error case: Token id is the same (not direct predecessor)")
                else:
                    # token_id < self._last_token_id
                    # new token round? Did we miss the restart? wait for receiving stuff from the leader
                    rospy.logwarn("Did we miss the restart? Waiting for reveiving the stuff from the leader")
            # else:
                # sender_id > (self._robot_id - 1)
                # sender_id > self._robot_id
                # do not accept, since the message came from a successor
                # rospy.logdebug("Ignore case: Message from a successor")

    def handle_command(self, sender_id, commandmsg):
        if commandmsg.command == "Turn" and self._robot_id == 1:
            rospy.loginfo("Execute turn command")
            self.publish_dir_command(commandmsg)
            self.change_driving_behavior(behaviors.FOLLOW_LINE)
        if commandmsg.command == "Turn" and self._robot_id == 2:
            self.change_driving_behavior(behaviors.FOLLOW_BLOB, 0)
            
            

    

def demo():
    global app 
    if not rospy.has_param('~id'):
        rospy.logerr("The parameter id is not defined.")
    else:
        ## The id of this robot read from configuration or console input
        robot_id = rospy.get_param('~id')
        rospy.loginfo("Launching robot with id: " + str(robot_id))

        if robot_id == 0:
            app = Leader(robot_id)  # implicit role
        else:
            app = Follower(robot_id)

    rospy.spin()
            
def demo1():
    int_ctrl = InternalController(robot_id=0)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        int_ctrl.publish_dir_command("left")
        rate.sleep()

def demo_switch(x):
    return {
        0: demo,
        1: demo1,
    }[x]

if __name__ == '__main__':
    rospy.init_node('demo_app')

    demoid = rospy.get_param('~demo')
    demo_switch(demoid)()
    #demo()
