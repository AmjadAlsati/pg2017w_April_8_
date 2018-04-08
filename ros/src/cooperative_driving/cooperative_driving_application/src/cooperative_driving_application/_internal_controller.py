"""!@package cooperative_driving_application
Defines the for the internal communication class.
"""

from __future__ import division
from builtins import range
from builtins import object
import functools

import rospy
from geometry_msgs.msg import Twist
from cooperative_driving_common import Pubscriber
from cooperative_driving_hardware.msg import LedCommand
from cooperative_driving_hardware.msg import LedValue
from cooperative_driving_msgs.msg import Command
from cooperative_driving_msgs.msg import Directions
from cooperative_driving_logic.srv import ChangeState
from cooperative_driving_networking.msg import EmergencyBrake
# constants

## In seconds, the time interval for flashing the LEDs
LED_DURATION = 3.0
## In seconds, the time to sleep between two LED flashes
LED_SEQUENCE_SLEEP = 0.2
## In seconds, the time the LEDs are on in the sequence
LED_SEQUENCE_LED_DURATION = 0.2

## A multiplier to approximate one round trip time between two robots
TRANSMISSION_OVERHEAD_MULTIPLIER = 2.0
## In seconds, the time for one round trip between two robots >= (RX + LED_DURATION + TX)
ONE_RTT = LED_DURATION * TRANSMISSION_OVERHEAD_MULTIPLIER

class InternalController(object):
    """!
    Internal Controller class for applications.

    Provides functionality for:
        * Control of internal factors via driving logic module
        * Actions to control speed, direction, route etc.
    """
    def __init__(self, robot_id):
        """!
        Initialization of a BaseApplication

        @param robot_id: The id of this robot given as a parameter during initialization
        """
        ##rospy.init_node("dynamic_tutorials", anonymous = True)

        ## Initialize publisher for DirectionCommand 
        ## Use next_action topic as an update channel for cooperative_driving_logic 
        self._dir_pub = rospy.Publisher('next_action', Command, queue_size=1)

        ## Publisher for LEDCommands
        self._led_pub = rospy.Publisher('/led_controller/cmd_led', LedCommand,
                                        queue_size=10)                                        

        ## The name of the first LED (color)
        self._led1_name = 'led/red'
        ## The name of the second LED (color)
        self._led2_name = 'led/blue'
        ## The value of the first LED (color)
        self._led1_value = 0
        ## The value of the second LED (color)
        self._led2_value = 0

        self.turn_off_leds()

        ## This robot's id
        self._robot_id = robot_id

        ## Subscriber for velocities from the drive controller
        self._vel_sub = rospy.Subscriber('/drive_controller/cmd_vel', Twist, self.store_latest_velocity)

        self._turn_sub = rospy.Subscriber('/crossing/directions', Directions, self.store_next_turn)

        ## This robot's latest velocity value
        self._velocity = 0.0
        ## This robot's latest steering value
        self._steering = 0.0

        ## Subscriber for internal emergency brake messages
        ## FIXME handle_emergency_brake_internal undefined
        #self._emergency_brake_internal_sub = rospy.Subscriber('/robot/emergency_brake', EmergencyBrake, self.handle_emergency_brake_internal, queue_size=10)

    def change_driving_behavior(self, driving_state, tag_id_to_follow=0):
        """!
        Sets the driving state in the logic

        @param driving_state: The driving state which is to be set in the logic. This is dependent on the robot's role
        """

        rospy.wait_for_service('/reflekte/change_state')
        try:
            change_state = rospy.ServiceProxy('/reflekte/change_state', ChangeState)
            change_state(target_state=driving_state, tag_id_to_follow=tag_id_to_follow)
            rospy.loginfo("Changed the driving state to " + str(driving_state))

        except rospy.ServiceException as e:
            rospy.logerr("change_state service call with " + str(driving_state) + " failed: " + str(e))
    
    def flash_leds(self, func=None):
        """!
        Sets the LED values, publishes a LedCommand and starts a led timer

        @param func: A function pointer which has to be called when the LEDs are turned off again
        """

        self._led1_value = 1
        self._led2_value = 1

        rospy.logdebug("Flash LEDs (" + str(self._led1_value) + ";" + str(self._led2_value) + ")")

        self.publish_led_command()

        # Set one-shot led timer
        #rospy.Timer(rospy.Duration(LED_DURATION), functools.partial(self.handle_led_timeout, func), True)
        # FIXME: Make LED blinking a service, such that the function call blocks
        rospy.sleep(rospy.Duration(LED_DURATION))

    def flash_leds_sequence(self):
        """!
        Flashes the LEDs in a sequence.
        Sets the values, publishes LedCommands and handles a LED timeout afterwards.
        """

        rospy.logdebug("Flashing LEDs in sequence")

        for _ in range(int(LED_DURATION / (LED_SEQUENCE_SLEEP + LED_SEQUENCE_LED_DURATION))):
            self._led1_value = 1
            self._led2_value = 1
            self.publish_led_command()

            rospy.sleep(LED_SEQUENCE_LED_DURATION)

            self._led1_value = 0
            self._led2_value = 0
            self.publish_led_command()

            rospy.sleep(LED_SEQUENCE_SLEEP)

    def handle_led_timeout(self, func=None, event=None):
        """!
        Callback for LED timer.
        Turn off the LEDs and calls func.

        @param func: A function pointer which shall be executed additionally to switching off the LEDs
        @param event: An event which is forwarded from the caller according to the ROS API
        """
        self.turn_off_leds()
        if func is not None:
            func()

    def turn_off_leds(self):
        """!
        Sets the LED values and publishes a LedCommand
        """

        self._led1_value = 0
        self._led2_value = 0

        rospy.logdebug("Turn off LEDs (" + str(self._led1_value) + ";" + str(self._led2_value) + ")")

        self.publish_led_command()

    def publish_led_command(self):
        """!
        Constructs and publishes LedCommand message
        """
        led_cmd = LedCommand()
        led_cmd.header.stamp = rospy.Time.now()
        led_1_value = LedValue()
        led_2_value = LedValue()
        led_1_value.value = self._led1_value
        led_2_value.value = self._led2_value
        led_1_value.frame_id = self._led1_name
        led_2_value.frame_id = self._led2_name
        led_cmd.values = [led_1_value, led_2_value]

        rospy.logdebug("Publishing LedCommand message: " + str(led_cmd))
        self._led_pub.publish(led_cmd)

    def publish_dir_command(self, command):

        rospy.logdebug("Publishing DirectionCommand message: ")
        self._dir_pub.publish(command)

    def store_latest_velocity(self, message):
        """!
        Callback for storing the latestd

        @param message: A cmd_vel message which in contins the latest velocity and steering values
        """

        self._velocity = message.linear.x
        self._steering = message.angular.z

    def store_next_turn(self, message):
        """!
        Store the availability of a turn and announce it to the follower

        @param message: A Direction msg to announce the available turn
        """
        for direction in message.directions:
            if direction.direction == 'RIGHT':
                self._lastTurn = direction.direction
                self.publish_command("Turn", direction)
            if direction.direction == 'LEFT':
                self._lastTurn = direction.direction
                self.publish_command("Turn", direction)

            
