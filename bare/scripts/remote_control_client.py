#! /usr/bin/env python2

"""This scripts implements a client for remotely controling a robot.

It continously waits for input from the gamepad which will be
transferred into drive instructions and then sent to the robot.
"""

from __future__ import division, print_function
import argparse
import logging
import struct

import pygame
import zmq
import time

from cooperative_driving.util import quitting

_GAMEPADS = {
    'DragonRise Inc.   Generic   USB  Joystick  ': {'_SPEED_AXIS': 3, '_STEER_AXIS': 1, '_SPEED_INVERSE':
                                                    True, '_STEER_INVERSE': False},
    'Microsoft SideWinder Game Pad Pro USB version 1.0': {'_SPEED_AXIS': 0, '_STEER_AXIS': 1, '_SPEED_INVERSE':
                                                          True, '_STEER_INVERSE': False}
}


def get_speed(gamepad):
    gamepad_dict = _GAMEPADS[gamepad.get_name()]
    value = gamepad.get_axis(gamepad_dict['_SPEED_AXIS'])
    if gamepad_dict['_SPEED_INVERSE']:
        return -value
    return value


def get_steer(gamepad):
    gamepad_dict = _GAMEPADS[gamepad.get_name()]
    value = gamepad.get_axis(gamepad_dict['_STEER_AXIS'])
    if gamepad_dict['_STEER_INVERSE']:
        return -value
    return value


def _init_gamepad(gamepad):
    gamepad.init()
    if gamepad.get_numaxes() < 2:
        raise ValueError("Gamepad has too few axes. (2 requried)")
    logging.info("Found gamepad %s with %d axes and %d buttons." % (gamepad.get_name(), gamepad.get_numaxes(), gamepad.get_numbuttons()))

def _init_connection(zmq_context, host, port):
    logging.info("Connecting to %s:%d" % (host, port))
    socket = zmq_context.socket(zmq.PAIR)
    socket.connect("tcp://%s:%d" % (host, port))
    return socket

def run_robot(host="localhost", port=42783, sample_rate=10):
    """Periodically send sinstructions to the robot."""
    logging.debug("Initializing pygame.")

    pygame.init()

    logging.debug("Initialized pygame.")

    pygame.joystick.init()
    number_gamepads = pygame.joystick.get_count()
    if number_gamepads < 1:
        logging.error("Did not find any gamepad. Please connect a gamepad.")
        return

    with quitting(pygame.joystick.Joystick(0)) as gamepad, zmq.Context() as context:
        _init_gamepad(gamepad)
        socket = _init_connection(context, host, port)

        running = True
        while running:
            pygame.event.pump()
            speed = get_speed(gamepad)
            steer = get_steer(gamepad)

            logging.debug("Setting speed to %f and sterring to %f." % (speed, steer))
            socket.send(struct.pack('!dd', speed, steer))

            time.sleep(1 / sample_rate)

        socket.close()
        context.term()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-v', '--verbosity', default='INFO', choices=('INFO', 'DEBUG', 'WARNING', 'ERROR'))
    parser.add_argument('-H', '--host', default='localhost', help='The host to connect to.')
    parser.add_argument('-p', '--port', default=42783, type=int, help='The port to connect on.')
    args = parser.parse_args()

    logging.getLogger().setLevel(args.verbosity)

    run_robot(args.host, args.port)


if __name__ == '__main__':
    main()
