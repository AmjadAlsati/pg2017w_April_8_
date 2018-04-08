from builtins import str
from builtins import chr
from builtins import object
#!/usr/bin/env python

import time
import smbus
from struct import unpack

###########################
# SETTINGS                #
###########################
CCS_I2C_BUS = 0X69

# instruction definitions
CCS_INSTR_SET_LED = 0X20
CCS_INSTR_SET_VELO = 0X1A

CCS_INSTR_GET_POS = 0X1B
CCS_LEN_GET_POS = 6

CCS_INSTR_GET_SENS = 0X10
CCS_LEN_GET_SENS = 13

# max and min constants
MAX_RAW_SPEED = 29
STOP_RAW_SPEED = 0
MIN_RAW_SPEED = -29  # -0x7f

MAX_STEER = 0.50

LED_RAW_MAX = 63

MIN_RAW_DISTANCE = 0
MAX_RAW_DISTANCE = 1024


class Device(object):

    def __init__(self, **kwds):
        "Initialize the smbus"
        self.bus = smbus.SMBus(1)
        print("bus initialized")

    def set_engine(self, speed, steer, **kwds):
        """
        set the engine according to the specified parameters. expected value for speed and steer are
        [-1.0,1.0]
        """
        speed = float(speed)
        steer = float(steer)
        # normalize values
        norm_steer = float(MAX_STEER) * steer
        norm_speed = speed - abs(norm_steer) * speed
        # calculate left/right velocity
        left_velo = norm_speed + norm_steer
        right_velo = norm_speed - norm_steer
        # calculate raw values for left/right wheel
        left_raw_velo = int(MAX_RAW_SPEED * left_velo)
        right_raw_velo = int(MAX_RAW_SPEED * right_velo)
        # write values to smbus
        print("velo: " + str(left_raw_velo) + "|" + str(right_raw_velo))
        self.bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [
                                      right_raw_velo, left_raw_velo])

    def set_led(self, red_led, blue_led, **kwd):
        """
        set the robots led according to the specified parameters. expected values for red and blue leds are [0.0, 1.0]
        """
        red_led = float(red_led)
        blue_led = float(blue_led)
        # check limits
        if red_led > 1.0:
            red_led = 1.0
        elif red_led < 0.0:
            red_led = 0.0

        if blue_led > 1.0:
            blue_led = 1.0
        elif blue_led < 0.0:
            blue_led = 0.0

        # calculate raw values
        raw_red_led = int(LED_RAW_MAX * red_led)
        raw_blue_led = int(LED_RAW_MAX * blue_led)

        # write values to smbus
        self.bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [
                                      raw_red_led, raw_blue_led])

    def get_pos(self):
        """
        figure out
        """
        result = self.bus.read_i2c_block_data(
            CCS_I2C_BUS, CCS_INSTR_GET_POS, CCS_LEN_GET_POS)

        xpos, ypos, orient = unpack('hhh', "".join([chr(x) for x in result]))
        return (xpos, ypos, orient)

    def get_sensors(self):
        """
        gets the sensor data
        """
        ret = self.bus.read_i2c_block_data(
            CCS_I2C_BUS, CCS_INSTR_GET_SENS, CCS_LEN_GET_SENS)
        result = ret[2:7]
        return result
        # front_right, front, front_left, rear_right, rear_lefxt = unpack(
        #   'hhhh', "".join([chr(x) for x in result]))
        # return (front_right, front, front_left, rear_right, rear_left)


if __name__ == "__main__":
    test_device = Device()
    # init pygame
    import pygame
    pygame.init()
    #pygame.display.set_mode((1, 1))
    # initialize joystick
    joy = pygame.joystick.Joystick(0)
    joy.init()
    ax_num = joy.get_numaxes()
    but_num = joy.get_numbuttons()

    if ax_num < 2:
        exit(0)

    sleep_time = 0.1
    led_switch = [0.0, 0.0]
    button_history = [0, 0, 0, 0]
    # pygame loop
    try:
        while (True):
            pygame.event.pump()
            if joy.get_button(4) != 0:
                if not button_history[0]:
                    led_switch[0] += 0.1
                    button_history[0] = 1
            else:
                button_history[0] = 0
            if joy.get_button(5) != 0:
                if not button_history[1]:
                    led_switch[0] -= 0.1
                    button_history[1] = 1
            else:
                button_history[1] = 0
            if joy.get_button(7) != 0:
                if not button_history[2]:
                    led_switch[1] += 0.1
                    button_history[2] = 1
            else:
                button_history[2] = 0
            if joy.get_button(6) != 0:
                if not button_history[3]:
                    led_switch[1] -= 0.1
                    button_history[3] = 1
            else:
                button_history[3] = 0

            if led_switch[0] < 0.0:
                led_switch[0] = 0.0
            if led_switch[0] > 1.0:
                led_switch[0] = 1.0
            if led_switch[1] < 0.0:
                led_switch[1] = 0.0
            if led_switch[1] > 1.0:
                led_switch[1] = 1.0

            test_device.set_engine((-joy.get_axis(1)), joy.get_axis(0))
            test_device.set_led(led_switch[0], led_switch[1])
            print("pos:" + str(test_device.get_pos()))
            print("sensors: " + str(test_device.get_sensors()))
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        joy.quit()
