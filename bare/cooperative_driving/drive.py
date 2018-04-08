"""
    Class CCSDev:
        Driver for controllin' the CCS robot platform by i2c (smbus)
        !!! works only on Raspi !!!
        on other platforms (set raspi = False) some values were returned
        drive: left, right
        led brightness: blue, red
        get position: x, y, angle
        distance messure: left, right, center (ch0, ch1, ch2)
"""
from builtins import chr
from builtins import object
import logging
from struct import unpack

import numpy as np

import smbus

import rospy

_CCS_I2C_ADDRESS = 0x69

_INSTRUCTION_SET_LED = 0x20
_INSTRUCTION_SET_VELOCITY = 0x1A

_INSTRUCTION_GET_POSITION = 0x1B
_GET_POSITION_LENGTH = 6

_INSTRUCTION_GET_SENSORS = 0x10
_GET_SENSOR_LENGTH = 13

# define the raw values for the control values
_MAX_RAW_SPEED = 50  # should be 127 in theory
_MAX_STEER = 0.50

_LED_RAW_MAX = 63


class CCSDev(object):
    """
    Control class for the CCS robot using i2c bus class use i2c on IO-header (3.3V???)
    """

    def _recv_callback(self, msg):
        """
        Callback function which is called whenever a new message is received on topic "cmd_vel".

        :param msg: Received message as geometry_msgs/Twist object (Vector3  linear, Vector3  angular)
        """
        print("Received message: {}, {}".format(msg.linear, msg.angular))

    def _publish_state_callback(self):
        """
        Callback function which is called periodically to publish the current odometry information.
        Message type: nav_msgs/Odometry
        Topic: odom
        """
        msg = nav_msgs.msg.Odometry()
        self._pub.publish(msg)


    def __init__(self, phony=False):
        """
        CCSDev.__init__()
        """
        self.__drive = (.0, .0)
        self.__leds = (.0, .0)

        rospy.init_node('velocity_sub', anonymous=True)
        rospy.Subscriber('cmd_vel', geometry_msgs.msg.Twist, self._recv_callback)
        rospy.spin()

        self._pub = rospy.Publisher('odom', nav_msgs.msg.Odometry, queue_size=1)
        rospy.init_node('state_pub', anonymous=True)
        rospy.Timer(rospy.Duration(2), self._publish_state_callback)

        self._phony = phony
        self._i2c_bus = None
        if not phony:
            try:
                self._i2c_bus = smbus.SMBus(1)
                self._i2c_bus.write_quick(_CCS_I2C_ADDRESS)
            except (OSError, IOError):
                logging.warning("No microcontroller detected.")
                self._phony = True
                if self._i2c_bus is not None:
                    self._i2c_bus.close()

    @property
    def drive(self):  # FIXME better name.
        """Return the current drive parameters, speed and steering."""
        return self.__drive

    @drive.setter
    def drive(self, (speed, steer)):
        """Set the drive's speed and steering.

        :param speed: The speed in range [-1, 1]
        :param steer: The steering in range [-1, 1]
        """
        speed = np.clip(speed, -1, 1)
        steer = np.clip(steer, -1, 1)

        # calculate limit for turns
        normalized_steer = float(_MAX_STEER) * float(steer)
        normalaized_speed = speed - abs(normalized_steer) * speed

        # calculate left /right velocity
        left_velocity = int(_MAX_RAW_SPEED * (normalaized_speed - normalized_steer))
        right_velocity = int(_MAX_RAW_SPEED * (normalaized_speed + normalized_steer))

        if not self._phony:
            self._i2c_bus.write_i2c_block_data(_CCS_I2C_ADDRESS, _INSTRUCTION_SET_VELOCITY, [
                left_velocity, right_velocity])
        else:
            logging.debug("Setting velocities to (%f, %f)." % (left_velocity, right_velocity))
        self.__drive = (speed, steer)

    @property
    def led(self):
        """The brightness of the red and blue LED."""
        return self.__leds

    @led.setter
    def led(self, (red, blue)):
        """Set LED brightness.

        :param red: Brightness of the red led in range [0, 1]
        :param blue: Brightness of the red led in range [0, 1]
        """
        red = np.clip(red, -1, 1)
        blue = np.clip(blue, -1, 1)
        raw_red = int(_LED_RAW_MAX * red)
        raw_blue = int(_LED_RAW_MAX * blue)
        if not self._phony:
            self._i2c_bus.write_i2c_block_data(_CCS_I2C_ADDRESS, _INSTRUCTION_SET_LED, [
                raw_red, raw_blue])
        else:
            logging.debug("Setting red/blue LEDs brightness to  to %f, %f." % (raw_red, raw_blue))
        self.__leds = (red, blue)

    @property
    def position(self):
        """Returns the estimated position of the robot.

        :returns: A tuple with x and y coordinates and the orientation.
        """
        if not self._phony:
            result = self._i2c_bus.read_i2c_block_data(
                _CCS_I2C_ADDRESS, _INSTRUCTION_GET_POSITION, _GET_POSITION_LENGTH)
            xpos, ypos, orientation = unpack(
                'hhh', "".join([chr(x) for x in result]))
            return (xpos, ypos, orientation)
        else:
            return (0, 0, 0)

    @property
    def sensors(self):
        """Returns the readings of the distance sensors.

        :returns: A tuple of distances in the range [0, 1] for the sensors in this order:
        front_right, front, front_left, rear_right, rear_left
        """
        if not self._phony:
            ret = self._i2c_bus.read_i2c_block_data(
                _CCS_I2C_ADDRESS, _INSTRUCTION_GET_SENSORS, _GET_SENSOR_LENGTH)
            # get the distance values out of the ret data
            result = ret[:5]
            front_right, front, front_left, rear_right, rear_left = unpack(
                'hhhhh', "".join([chr(x) for x in result]))
            return (front_right, front, front_left, rear_right, rear_left)
        else:
            return (0, 0, 0, 0, 0)

    def close(self):
        """Closes any resources held by the device."""
        if not self._phony:
            logging.debug("Stopping device.".format(self._i2c_bus))
            self._i2c_bus.write_i2c_block_data(_CCS_I2C_ADDRESS, _INSTRUCTION_SET_VELOCITY, list((0, 0)))
            self._i2c_bus.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
