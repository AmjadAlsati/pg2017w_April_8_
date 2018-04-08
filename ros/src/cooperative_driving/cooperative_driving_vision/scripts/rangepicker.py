#!/usr/bin/python2

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

bridge = CvBridge()


def noop(*args, **kwargs):
    pass


def get_bgr_range():
    return ((cv2.getTrackbarPos('B1', 'frame'),
             cv2.getTrackbarPos('G1', 'frame'),
             cv2.getTrackbarPos('R1', 'frame'), 0),
            (cv2.getTrackbarPos('B2', 'frame'),
             cv2.getTrackbarPos('G2', 'frame'),
             cv2.getTrackbarPos('R2', 'frame'), 0))


def get_hsv_range():
    return ((cv2.getTrackbarPos('H1', 'frame'),
             cv2.getTrackbarPos('S1', 'frame'),
             cv2.getTrackbarPos('V1', 'frame'), 0),
            (cv2.getTrackbarPos('H2', 'frame'),
             cv2.getTrackbarPos('S2', 'frame'),
             cv2.getTrackbarPos('V2', 'frame'), 0))


def range_picker(image_msg):
    frame = bridge.imgmsg_to_cv2(image_msg)
    # frame = cv2.cvtColor(img, code=cv2.COLOR_RGB2HSV)
    if frame is not None:
        cv2.imshow('frame', cv2.inRange(frame, *get_bgr_range()))
        # cv2.imshow('frame', cv2.inRange(frame, *get_hsv_range()));
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return


def main():

    rospy.init_node('rangepicker', anonymous=True)

    cv2.namedWindow('frame')

    cv2.createTrackbar('B1', 'frame', 0, 255, noop)
    cv2.createTrackbar('G1', 'frame', 0, 255, noop)
    cv2.createTrackbar('R1', 'frame', 0, 255, noop)

    cv2.createTrackbar('B2', 'frame', 180, 255, noop)
    cv2.createTrackbar('G2', 'frame', 255, 255, noop)
    cv2.createTrackbar('R2', 'frame', 255, 255, noop)

    cv2.createTrackbar('H1', 'frame', 0, 180, noop)
    cv2.createTrackbar('S1', 'frame', 0, 255, noop)
    cv2.createTrackbar('V1', 'frame', 0, 255, noop)

    cv2.createTrackbar('H2', 'frame', 180, 180, noop)
    cv2.createTrackbar('S2', 'frame', 255, 255, noop)
    cv2.createTrackbar('V2', 'frame', 255, 255, noop)

    rospy.Subscriber("/camera/image_raw", Image, range_picker, queue_size=1, buff_size=2 ** 20)

    rospy.spin()


if __name__ == '__main__':
    main()
