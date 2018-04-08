#!/usr/bin/env python

"""! @package led_controller_pi
Controller for additional RaspberryPi LEDs.
"""

import rospy
import atexit
from cooperative_driving_hardware.msg import LedCommand
import RPi.GPIO as GPIO

## Pin address of first LED (red)
LED_PIN_1 = 20
## Pin address of second LED (blue)
LED_PIN_2 = 21


class Pi_LED_Controller:
    """!
    Controller for additional RaspberryPi LEDs.

    Listens to the same topic as the normale LedController.
    """

    def __init__(self):
        """!
        Initialization of the controller.
        """

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(LED_PIN_1, GPIO.OUT)
        GPIO.setup(LED_PIN_2, GPIO.OUT)

        # Subscriber for receiving LedCommand messages
        rospy.Subscriber('/led_controller/cmd_led', LedCommand, self.received_led_values)
        ## The value of the red LED
        self._led_red = 0
        ## The value of the blue LED
        self._led_blue = 0
        self.control_leds()

    def delete(self):
        """!
        Is called on exit to cleanly reset the LEDs
        """

        self._led_red = 0
        self._led_blue = 0
        self.control_leds()

    def control_leds(self):
        """!
        Sets the values of the LEDs
        """

        # Control leds and show logging message
        GPIO.output(LED_PIN_1, int(self._led_red))
        rospy.logdebug("Pi-LED-Controller: red led %d", self._led_red)

        GPIO.output(LED_PIN_2, int(self._led_blue))
        rospy.logdebug("Pi-LED-Controller: blue led %d", self._led_blue)

    def received_led_values(self, led_values):
        """!
        Call-back for receiving a LedCommand message.

        Sets new values for both LEDs.
        """

        rospy.logdebug("Received packet: " + str(led_values))
        rospy.logdebug(str(led_values.values))

        for value in led_values.values:
            if value.frame_id == "led/red":
                self._led_red = int(value.value)
            elif value.frame_id == "led/blue":
                self._led_blue = int(value.value)

        self.control_leds()

if __name__ == '__main__':
    #  rospy.init_node('pi_led_controller', log_level=rospy.DEBUG)
    rospy.init_node('pi_led_controller')
    app = Pi_LED_Controller()
    atexit.register(app.delete)

    rospy.spin()
