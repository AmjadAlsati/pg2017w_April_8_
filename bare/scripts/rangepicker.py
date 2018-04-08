#!/usr/bin/python2

import cv2

from cooperative_driving.camera import create as create_camera
from cooperative_driving.util import noop


def get_hsv_range():
    return ((cv2.getTrackbarPos('H1', 'image'),
             cv2.getTrackbarPos('S1', 'image'),
             cv2.getTrackbarPos('V1', 'image'), 0),
            (cv2.getTrackbarPos('H2', 'image'),
             cv2.getTrackbarPos('S2', 'image'),
             cv2.getTrackbarPos('V2', 'image'), 0))


def main():
    cv2.namedWindow('image')

    cv2.createTrackbar('H1', 'image', 0, 180, noop)
    cv2.createTrackbar('S1', 'image', 0, 255, noop)
    cv2.createTrackbar('V1', 'image', 0, 255, noop)

    cv2.createTrackbar('H2', 'image', 180, 180, noop)
    cv2.createTrackbar('S2', 'image', 255, 255, noop)
    cv2.createTrackbar('V2', 'image', 255, 255, noop)

    try:
        with create_camera() as camera:
            while True:
                img = cv2.cvtColor(camera.capture(), code=cv2.COLOR_RGB2HSV)
                if img is not None:
                    cv2.imshow('image', cv2.inRange(img, *get_hsv_range()))
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
