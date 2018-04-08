#!/usr/bin/python2

import argparse
import os
import sys

import rospy
import cv_bridge
import sensor_msgs
import cv2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('topic', help="The broadcast topic.")
    parser.add_argument('image', help="The image to be broadcast.")
    parser.add_argument('-r', '--rate', type=int, default=1, help="The broadcasting rate.")
    args = parser.parse_args()

    rospy.init_node('dummy_camera', anonymous=True)
    if not os.path.exists(args.image):
        rospy.logerr("Image does not exist.")
        sys.exit(1)

    rate = rospy.timer.Rate(args.rate)
    image_publisher = rospy.Publisher(args.topic, sensor_msgs.msg.Image, queue_size=1)
    bridge = cv_bridge.CvBridge()
    image = cv2.imread(args.image)

    while not rospy.is_shutdown():
        image_publisher.publish(bridge.cv2_to_imgmsg(image, 'bgr8'))
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()
