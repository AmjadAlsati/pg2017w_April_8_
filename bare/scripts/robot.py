#!/usr/bin/python2
import argparse
import logging
import re

from cooperative_driving.robot import Robot
from cooperative_driving.util import on_raspi

if on_raspi():
    _COLORS = {'red': ((128, 158, 0), (180, 255, 255)),  # big ball
               'green': ((42, 159, 73), (70, 255, 206)),  # big ball
               #'green': ((55, 218, 72), (81, 255, 227))  # small ball
               'orange': ((0, 117, 137), (19, 255, 255)),
               'blue': ((60, 96, 94), (180, 255, 239)),
               'yellow': ((29, 206, 85), (46, 255, 255)),
               'pink': ((36, 122, 116), (180, 255, 255)),  # TODO calibrate
               'purple': ((102, 49, 41), (180, 100, 112)), }
else:
    _COLORS = {'red': ((20, 56, 40), (61, 82, 67)),  # big ball
               'green': ((14, 76, 34), (95, 184, 102)),  # big ball
               #'green': ((55, 218, 72), (81. 255, 227))  # small ball
               'orange': ((0, 119, 148), (90, 231, 255)),
               'blue': ((60, 96, 94), (180, 255, 239)),
               'yellow': ((29, 206, 85), (46, 255, 255)),
               'pink': ((108, 42, 120), (172, 111, 255)),
               'purple': ((102, 0, 2), (180, 255, 189)), }


def color_range(string):
    if string in set(_COLORS.keys()):
        return _COLORS[string]
    elif re.match("^0x[0-9A-Fa-f]{6},0x[0-9A-Fa-f]{6}$", string):
        return tuple(tuple(int(x[i:i + 2], 16) for i in range(2, 8, 2)) for x in string.split(','))
    else:
        raise argparse.ArgumentError("Invalid color range.")


def main():  # TODO add subparsers for the different states
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-v', '--verbosity', default='DEBUG',
                        choices=('INFO', 'DEBUG', 'WARNING', 'ERROR'))
    parser.add_argument('-p', '--port', type=int, default=42783, help='The port to listen to connections on.')
    subparsers = parser.add_subparsers(dest='state', title="States", help="The robot's state.")

    # idle
    subparsers.add_parser('idle', help="Idle state.")

    # follow line
    subparsers.add_parser('line', help="Line following state.")

    # follow blob
    parser_blob = subparsers.add_parser('blob', help="Follow a colored ball.")
    parser_blob.add_argument('color_range', type=color_range,
                        help="The color range to track. Must either be one of %s, or the comma separated HSV values of a color range (e.g. 0x000000,0xABCDEF)" % set(_COLORS.keys()))

    # remote control
    subparsers.add_parser('rc', help="Use inputs send via network.")

    # turn
    parser_turn = subparsers.add_parser('turn', help="Turn the robot.")
    parser_turn.add_argument('-d', '--direction', default='left', choices=('left', 'right'), help="The direction to turn to.")
    parser_turn.add_argument('degrees', nargs='?', type=int, default=180, help="The number of degrees to turn.")

    args = parser.parse_args()

    logging.getLogger().setLevel(args.verbosity)

    robot_params = vars(args)
    del robot_params['verbosity']

    with Robot(**robot_params) as robot:
        robot.run()


if __name__ == '__main__':
    main()
