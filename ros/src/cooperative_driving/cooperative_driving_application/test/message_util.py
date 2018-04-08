#!/usr/bin/env python2

import unittest

import rospy

from cooperative_driving_application import (
    create_cooperative_awareness_message,
    create_emergency_braking_message,
    create_platooning_message,
    create_token_message
)

PKG = 'cooperative_driving_application'
NAME = 'message_util'


class TestMessageUtil(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.rostime.set_rostime_initialized(True)
        rospy.rostime._set_rostime(rospy.Time())

    def test_token_message(self):
        token_msg = create_token_message(1, 1)
        self.assertTrue(token_msg.sender_id == 1)
        self.assertTrue(token_msg.token_id == 1)

    def test_emergency_brake_message(self):
        emergency_braking_msg = create_emergency_braking_message(True)
        self.assertTrue(emergency_braking_msg.sender_id == 1)
        self.assertTrue(emergency_braking_msg.enable is True)

    def test_platooning_message(self):
        platooning_msg = create_platooning_message(1)
        self.assertTrue(platooning_msg.sender_id == 1)

    def test_cooperative_awareness_message(self):
        cooperative_awareness_msg = create_cooperative_awareness_message(1, 0.0, 0.0)
        self.assertTrue(cooperative_awareness_msg.sender_id == 1)
        self.assertTrue(cooperative_awareness_msg.velocity == 0.0)
        self.assertTrue(cooperative_awareness_msg.steering == 0.0)


if __name__ == "__main__":
    import rosunit

    rosunit.unitrun(PKG, NAME, TestMessageUtil)
