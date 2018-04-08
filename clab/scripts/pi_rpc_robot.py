#!/usr/bin/python2
################
#
# Modul:        rc_simple_local_robot.py
# File:         rc_simple_robot.py
#
# Author:       Bernd Kleinjohann
# Created:      Oct. 2016
# Version:      0.1
#
# Contents:

import argparse
import sys
from time import sleep

import optparse

from framework.smart_object import SMOBaseObject
from framework.robots.pi_rpc_robot import CCSCallRobot_1


def main():
    p = optparse.OptionParser()
    p.add_option('--time', '-T', default='30')

    options, arguments = p.parse_args()
    
    # create robot
    SMOBaseObject.debug_handler.out('+++ main start: Create robot')
    rob = CCSCallRobot_1(cam_index=0)

    SMOBaseObject.debug_handler.out('+++ main: Start robot')
    rob.start()

    s_time = int(options.time) # TODO insert run time here
    SMOBaseObject.debug_handler.out('+++ main robot runs for %d seconds' % s_time)
    sleep(s_time)

    SMOBaseObject.debug_handler.out('+++ main stop robot')
    rob.stop()
    # SMO_IP_delete_all_server()
    SMOBaseObject.debug_handler.out('+++ main exit')
    sys.exit()


if __name__ == '__main__':
    main()
