#!/usr/bin/python2
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$
import argparse
import struct

import rospy
from std_msgs.msg import String

_SIZE = 1024


def talker(size, topic, name='talker', rate=50):
    pub = rospy.Publisher(topic, String, queue_size=1)
    rospy.init_node(name, anonymous=True)
    rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        pub.publish(struct.pack('!d', rospy.get_time()) + ('\0' * max(size - 8, 0)))
        rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser("Simple publisher, sends timestamps followed by a configurable padding.")
    parser.add_argument('-r', '--rate', type=int, default=50, help="The rate at which messages are generated.")
    parser.add_argument('size', type=int, help="Message length in byte (at least 8).")
    parser.add_argument('-t', '--topic', type=str, help="Name of the topic (alphanumeric).")
    parser.add_argument('-n', '--name', type=str, default="talker", help="Name of the talker.")

    args = parser.parse_args()
    try:
        talker(args.size, args.topic, args.name, args.rate)
    except rospy.ROSInterruptException:
        pass
