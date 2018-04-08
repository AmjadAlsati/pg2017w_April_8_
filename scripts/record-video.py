#!/usr/bin/python2
from __future__ import print_function
import argparse
import logging

import picamera


def record_video(filename, recording_time=60):
    if filename is None:
        logging.error("The filename is empty, please specify one.")
        return
    logging.debug("Creating camera")
    with picamera.PiCamera() as camera:
        logging.debug("Created the camera")
        logging.debug("Setting resolution")
        camera.resolution = (1920, 1080)
        logging.debug("Set resoution to 1920x1080")
        logging.debug("Starting recording")
        camera.start_recording(filename + '.h264')
        logging.info("Started recording video to {} for {}s".format(filename, recording_time))
        camera.wait_recording(recording_time)
        logging.debug("Waited for {}s".format(recording_time))
        logging.debug("Stopping recording")
        camera.stop_recording()
        logging.info("Stopped recording")
    logging.info("Done")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-v', '--verbosity', default='INFO', choices=('INFO', 'DEBUG', 'WARNING', 'ERROR'))
    parser.add_argument('filename', type=str, default='video.mp4', help="The filename to store the video file to.")
    parser.add_argument('-t', '--recording_time', type=int, default=60)
    args = parser.parse_args()

    logging.getLogger().setLevel(args.verbosity)

    record_video(args.filename, args.recording_time)


if __name__ == '__main__':
    main()
