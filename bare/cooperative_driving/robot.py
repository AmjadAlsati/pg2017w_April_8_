"""This module contains definitions for a CCS-Robot."""
from __future__ import division
import collections
import logging
import math
import struct
import time

import cv2
import numpy as np
import zmq
from scipy.ndimage.filters import gaussian_filter

from .drive import  CCSDev
from .camera import create as create_camera

_Decision = collections.namedtuple('Decision', ('speed', 'steer'))


class Robot(object):
    """A `Robot` represents the software-side of a CCS-robot.

    Its behavior can be configured by setting different states.
    Depending on the current state the robot uses its actors,
    to accomplish the goal set by the state.
    """

    _Data = collections.namedtuple('Data', ('image',))

    # state specific variables, see TODOs
    _DEVIATION_FACTOR = 2.0
    _PAST_LENGTH = 15

    def __init__(self, state='idle', color_range=None, port=42783, *args, **kwd):
        self._state = state
        self._decider = _state_deciders[state](self, *args, **kwd)
        self._camera = create_camera()
        self._drive = CCSDev()
        self._zmq_context = zmq.Context()
        self._socket = self._zmq_context.socket(zmq.PAIR)
        self._socket.bind('tcp://0.0.0.0:%d' % port)
        self._running = False

        # state specific variables, see TODOs
        self._last_values = []  # line
        self._color_range = color_range  # blob

    def run(self):
        """Runs the robot."""

        self._running = True
        last_iteration = time.time()
        while self._running:
            data = self._get_data()
            features = self._extract_features(data.image)
            decision = self._decider.decide(data, features)
            self._apply_decision(decision)

            time_current = time.time()
            logging.info("Running main loop at %dHz rate" % (
                1 / (time_current - last_iteration)))
            last_iteration = time_current

    def stop(self):
        """Stops the robot's mainloop."""
        self._running = False

    def close(self):
        if self._running:
            self.stop()
        self._camera.close()
        self._drive.close()
        self._zmq_context.destroy()

    def _get_data(self):
        return self._Data(self._camera.capture())

    def _extract_features(self, image):  # TODO: extract features independent of state.
        if self._state == 'line':
            line = image[-5:, ...].sum(axis=2).mean(axis=0)
            line_location = gaussian_filter(line, .1 * image.shape[1]).argmax()
            return [line_location]
            new_values = (line.mean(), line.std(), line_location)
            if len(self._last_values) > self._PAST_LENGTH:
                deviation = (self._DEVIATION_FACTOR * np.std(self._last_values, axis=0) <
                             np.abs(np.mean(self._last_values, axis=0) - new_values))
                if deviation.all():
                    self._last_values.append(new_values)
                    return [line_location]
                else:
                    return [self._last_values[-1][2] if len(self._last_values) > .5 * self._PAST_LENGTH else 0]

                self._last_values = self._last_values[1:]
            else:
                self._last_values.append(new_values)
            return [line_location]
        elif self._state == 'blob':
            hsv_frame = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv_frame, *self._color_range)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                m = cv2.moments(c)
                center = (int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]))
                if radius > 10:
                    return [(center, radius)]
            return [None]

    def _apply_decision(self, decision):
        logging.debug("Setting speed %f, steer %f" % decision)
        self._drive.drive = decision

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


class _Decider(object):
    """Base class for robot behavioral description."""


    def __init__(self, robot):
        self._robot = robot

    def decide(self, data, features):
        """Compute a `_Decision` based on the robot's input.

        This is called once per mainloop after all input is computed.
        Based on the robot's input the subclasses should compute the
        decision for the next timestep.

        :param data: Raw sensor readings
        :param features: Features extracted from the image captured.
        """


class _IdleDecider(_Decider):
    """Does not change its position."""

    def decide(self, data, features):
        return _Decision(0, 0)

class _FollowLineDecider(_Decider):
    """Follows a bright lane on the ground."""

    SPEED_FACTOR = .7

    def decide(self, data, features):
        return _Decision(self.SPEED_FACTOR, 2 * (features[0] / data.image.shape[1]) - 1)


class _FollowBlobDecider(_Decider):
    """Follows a colored blob in front of the robot."""

    STEER_FACTOR = .5

    def decide(self, data, features, target_distance=.3, min_distance=-.2, max_distance=5):
        feature = features[0]
        if feature is not None:
            interp_x = [min_distance, .9 * target_distance, 1.1 * target_distance, .3 *
                        (max_distance - 1.1 * target_distance), max_distance]
            interp_y = [-.5, 0, 0, .7, 1]

            normalized_location = (2 * feature[0][0] / (data.image.shape[1] / 2)) - 1  # scale feature location to [-1, 1]
            distance = 1 / math.tan(2 * math.pi * feature[1] / data.image.shape[1])
            speed = np.interp(distance, interp_x, interp_y)
            return _Decision(speed, self.STEER_FACTOR * normalized_location)
        else:
            return _Decision(0, 0)

class _RemoteControlDecider(_Decider):
    """Follows instructions received on the robot's socket."""

    def decide(self, data, features):
        return _Decision(*struct.unpack('!dd', self._robot._socket.recv()))


class _TurnDecider(_Decider):
    """Turns the robot before shutting down."""

    def __init__(self, robot, direction='left', degrees=180):
        super(_TurnDecider, self).__init__(robot)
        self._direction = direction
        self._degrees = degrees
        self._start_time = None

    def decide(self, data, features, seconds_per_degree=7/900):
        if self._start_time is None:
            self._start_time = time.time()
        if (time.time() - self._start_time) < seconds_per_degree * self._degrees:
            return _Decision(0, -1 if self._direction == 'left' else 1)
        else:
            self._robot.stop()
            return _Decision(0, 0)


_state_deciders = {
    'idle': _IdleDecider,
    'line': _FollowLineDecider,
    'blob': _FollowBlobDecider,
    'rc': _RemoteControlDecider,
    'turn': _TurnDecider,
}
