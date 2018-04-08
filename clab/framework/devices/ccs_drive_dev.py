################
#
# Modul:        devices
# File:         ccs_drive_dev.py
#
# Author:       Bernd Kleinjohann
# Created:      Oct. 2016
# Version:      0.1
#
# Contents:

"""
    Class CCSDev:
        Driver for controllin the CCS robot platform by i2c (smbus)
        !!! works only on Raspi !!!
        on other platforms (set raspi = False) some values were returned
        drive: left, right
        led brightness: blue, red
        get position: x, y, angle
        distance messure: left, right, center (ch0, ch1, ch2)
"""
from builtins import str
from builtins import chr
from builtins import object

from time import sleep
from struct import unpack

from framework.smart_object import SMOBaseObject

raspi = True
# raspi = False

if raspi:
    import smbus

# debug options
drive_test = True
# drive_test = False
stick_test = True
# stick_test = False

# define i2c bus id
ccs_i2s_bus = 0x69

# define instructions
ccs_instr_set_led = 0x20
ccs_instr_set_velo = 0x1A

ccs_instr_get_pos = 0x1B
ccs_len_get_pos = 6

ccs_instr_get_sens = 0x10
ccs_len_get_sens = 13

# define the raw values for the control values
max_raw_speed = 29  # should be 127 in theory
stop_raw_speed = 0
min_raw_speed = -29  # should be -127 in theory
max_steer = 0.50

led_raw_max = 63

min_raw_distance = 0
max_raw_distance = 1024


class CCSDev(object):
    '''
    Control class for the CCS robot using i2c bus class use i2c on IO-header (3.3V???)
    '''

    def __init__(self, **kwd):
        """
        CCSDev.__init__()
        """
        self.left_velo = 0
        self.right_velo = 0
        if raspi:
            self.i2c_bus = smbus.SMBus(1)
        else:
            self.i2c_bus = None
        return

    def set_drive(self, speed, steer, **kwd):
        """
        CCSDev.set_drive(speed, steer) expect values from [-1.0, 1.0] returns None
        set max_steer (0.25) to define limits forrotation speed
        """
        # calculate the velocity of right and left wheel from speed [-1, 1] and
        # steer left, right [-1, 1]

        # assert foat parameter
        speed = float(speed)
        steer = float(steer)

        # calculate limit for turns
        norm_steer = float(max_steer) * float(steer)
        norm_speed = speed - abs(norm_steer) * speed
        if drive_test:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** CCSDev control drive %s' % str((norm_speed, norm_steer)))

        # calculate left /right velocity
        self.left_velo = norm_speed + norm_steer
        self.right_velo = norm_speed - norm_steer
        if drive_test:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** CCSDev set drive %s' % str((self.left_velo, self.right_velo)))

        # calculate the raw value for left and right wheel
        left_raw_velo = int(max_raw_speed * self.left_velo)
        right_raw_velo = int(max_raw_speed * self.right_velo)
        if drive_test:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** CCSDev set raw drive %s' % str((left_raw_velo, right_raw_velo)))

        if raspi:
            self.i2c_bus.write_i2c_block_data(ccs_i2s_bus, ccs_instr_set_velo, [
                                              right_raw_velo, left_raw_velo])
        else:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** CCSDev.set_drive send %s' % str((left_raw_velo, right_raw_velo)))
        return

    def get_drive(self):
        return (self.left_velo , self.right_velo)

    def set_led(self, red_led, blue_led, **kwd):
        """
        CCSDev.set_led(red_led, blue_led) expect values from [0.0, 1.0] returns None
        """
        # assert foat parameter
        red_led = float(red_led)
        blue_led = float(blue_led)

        #   set led values
        if red_led > 1.0:
            red_led = 1.0
        if red_led < 0.0:
            red_led = 0.0
        raw_red_led = int(led_raw_max * red_led)

        if blue_led > 1.0:
            blue_led = 1.0
        if blue_led < 0.0:
            blue_led = 0.0
        raw_blue_led = int(led_raw_max * blue_led)

        if drive_test:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** CCSDev set led %s' % str((red_led, blue_led)))
        if drive_test:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** CCSDev set raw led %s' % str((raw_red_led, raw_blue_led)))

        if raspi:
            self.i2c_bus.write_i2c_block_data(ccs_i2s_bus, ccs_instr_set_led, [
                                              raw_red_led, raw_blue_led])
        else:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** CCSDev.set_led send %s' % str((raw_red_led, raw_blue_led)))
        return

    def get_pos(self):
        """
        CCSDev.get_pos() returns(xpos, ypos, orient) values from [0.0, 1.0] returns None
        """
        if raspi:
            result = self.i2c_bus.read_i2c_block_data(
                ccs_i2s_bus, ccs_instr_get_pos, ccs_len_get_pos)
            return result
            #xpos, ypos, orient = unpack(
            #    'hhh', "".join([chr(x) for x in result]))
            if drive_test:
                SMOBaseObject.debug_handler.out(
                    '*** INFO *** CCSDev.get_pos returns %s' % str((xpos, ypos, orient)))
            return (xpos, ypos, orient)
        else:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** CCSDev.get_pos returns %s' % str((0, 0, 0)))
            return (0, 0, 0)

    def get_sensors(self):
        """
        CCSDev.get_sensors() returns(front_right, front, front_left, rear_right, rear_left) values from [0.0, 1.0]
        returns None
        """
        if raspi:
            ret = self.i2c_bus.read_i2c_block_data(
                ccs_i2s_bus, ccs_instr_get_sens, ccs_len_get_sens)
            # get the distance values out of the ret data
            result = ret[:5]
            front_right, front, front_left, rear_right, rear_left = unpack(
                'hhhhh', "".join([chr(x) for x in result]))
            if drive_test:
                SMOBaseObject.debug_handler.out('*** INFO *** CCSDev.get_sensors returns %s' % str(
                    (front_right, front, front_left, rear_right, rear_left)))
            return (front_right, front, front_left, rear_right, rear_left)
        else:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** CCSDev.get_sensors returns %s' % str((0, 0, 0, 0, 0)))
            return (0, 0, 0, 0, 0)


if __name__ == '__main__':
    SMOBaseObject.debug_handler.out('+++++++++++++++++++++++++++++++++++')
    SMOBaseObject.debug_handler.out('+++ Start Testsequence for CCSDev')
    SMOBaseObject.debug_handler.out('+++++++++++++++++++++++++++++++++++')

    test_dev = CCSDev()

    if stick_test:
        SMOBaseObject.debug_handler.out('+++ Start joystick test')
        import pygame
        pygame.init()
        joy = pygame.joystick.Joystick(0)
        joy.init()
        SMOBaseObject.debug_handler.out('+++ axes: ', joy.get_numaxes(), ' buttons: ',
                                        joy.get_numbuttons(), ' balls: ', joy.get_numballs(), ' hats: ', joy.get_numhats())
        test_cycles = 60
        sleep_time = 0.1
        SMOBaseObject.debug_handler.out(
            '+++ start cycles a %f seconds' % sleep_time)

        ax_no = joy.get_numaxes()
        if ax_no < 2:
            SMOBaseObject.debug_handler.out('+++ Found no axis')
            exit(0)

        but_no = joy.get_numbuttons()
        if ax_no < 4:
            SMOBaseObject.debug_handler.out('+++ Found not enough axis')
            exit(0)

        led_switch = [0.0, 0.0]
        button_history = [0, 0, 0, 0]
        try:
            while(True):
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

                test_dev.set_drive((-joy.get_axis(1)), joy.get_axis(0))
                test_dev.set_led(led_switch[0], led_switch[1])
                # test_dev.get_pos()
                # test_dev.get_sensors()

                sleep(sleep_time)
        except KeyboardInterrupt:
            joy.quit()
    else:
        SMOBaseObject.debug_handler.out('+++ Set drive 0.5, 0.5')
        test_dev.set_drive(0.5, 0.5)
        SMOBaseObject.debug_handler.out('+++ Set drive 0.5, -0.5')
        test_dev.set_drive(0.5, 0.5)
        SMOBaseObject.debug_handler.out('+++ Set drive 0, 0')
        test_dev.set_drive(0, 0)

        SMOBaseObject.debug_handler.out('+++ Set led 0.5, 0.5')
        test_dev.set_led(0.5, 0.5)

        SMOBaseObject.debug_handler.out('+++ Set led 1.0, 0.25')
        test_dev.set_led(1.0, 0.25)

        SMOBaseObject.debug_handler.out('+++ Set led 0, 0')
        test_dev.set_led(0, 0)

        ret = test_dev.get_pos()
        SMOBaseObject.debug_handler.out('+++ Get pos: ', str(ret))

        ret = test_dev.get_sensors()
        SMOBaseObject.debug_handler.out('+++ Get sensors: ', str(ret))

        SMOBaseObject.debug_handler.out('+++ Stop Testsequence for CCSDev')
        SMOBaseObject.debug_handler.out('+++++++++++++++++++++++++++++++++++')
