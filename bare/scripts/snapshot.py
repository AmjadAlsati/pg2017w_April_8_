#!/usr/bin/python2
import argparse
import logging

import cv2

from cooperative_driving.camera import create as create_camera


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-v', '--verbosity', default='DEBUG',
                        choices=('INFO', 'DEBUG', 'WARNING', 'ERROR'))
    parser.add_argument('path', help="The path to store the image at.")
    args = parser.parse_args()

    logging.getLogger().setLevel(args.verbosity)

    with create_camera() as camera:
        if cv2.imwrite(args.path, camera.capture()):
            logging.info("Saved image to %s." % args.path)


if __name__ == '__main__':
    main()
