#!/usr/bin/env python2
import time
import unittest

import rospy
from geometry_msgs.msg import PointStamped

from cooperative_driving_common import Pubscriber


class PubscriberTest(unittest.TestCase):

    def setUp(self):
        topic = 'topic'
        self._sub_callcount = 0
        self._pubsub_callcount = 0

        def sub_callback(msg):
            self._sub_callcount += 1

        def pubsub_callback(msg):
            self._pubsub_callcount += 1

        self._pubsub = Pubscriber(topic, PointStamped, pubsub_callback, input_queue_size=1, output_queue_size=1)
        self._pub = rospy.Publisher(topic, PointStamped, queue_size=1)
        self._sub = rospy.Subscriber(topic, PointStamped, sub_callback, queue_size=1)
        time.sleep(1e-1)

    def test_publish(self):
        self._pubsub.publish()
        time.sleep(1e-1)
        self.assertEqual(self._sub_callcount, 1)

    def test_publish_kwd(self):
        self._pubsub.publish(PointStamped())
        time.sleep(1e-1)
        self.assertEqual(self._sub_callcount, 1)

    def test_subscribe(self):
        self._pub.publish()
        time.sleep(1e-1)
        self.assertEqual(self._pubsub_callcount, 1)

    def test_no_self_receive(self):
        self._pubsub.publish()
        time.sleep(1e-1)
        self.assertEqual(self._pubsub_callcount, 0)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_pubscriber_python')
    rostest.rosrun('cooperative_driving_common', 'test_pubscriber_python', PubscriberTest)
