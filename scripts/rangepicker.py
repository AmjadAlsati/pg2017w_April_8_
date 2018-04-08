#!/usr/bin/python2

import sys

import cv2


def noop(*args, **kwargs):
    pass


def get_bgr_range():
    return ((cv2.getTrackbarPos('B1', 'Rangepicker'),
             cv2.getTrackbarPos('G1', 'Rangepicker'),
             cv2.getTrackbarPos('R1', 'Rangepicker'), 0),
            (cv2.getTrackbarPos('B2', 'Rangepicker'),
             cv2.getTrackbarPos('G2', 'Rangepicker'),
             cv2.getTrackbarPos('R2', 'Rangepicker'), 0))


def get_hsv_range():
    return ((cv2.getTrackbarPos('H1', 'Rangepicker'),
             cv2.getTrackbarPos('S1', 'Rangepicker'),
             cv2.getTrackbarPos('V1', 'Rangepicker'), 0),
            (cv2.getTrackbarPos('H2', 'Rangepicker'),
             cv2.getTrackbarPos('S2', 'Rangepicker'),
             cv2.getTrackbarPos('V2', 'Rangepicker'), 0))


def main():
    cv2.namedWindow('Rangepicker')

    cv2.createTrackbar('B1', 'Rangepicker', 0, 255, noop)
    cv2.createTrackbar('G1', 'Rangepicker', 0, 255, noop)
    cv2.createTrackbar('R1', 'Rangepicker', 0, 255, noop)

    cv2.createTrackbar('B2', 'Rangepicker', 255, 255, noop)
    cv2.createTrackbar('G2', 'Rangepicker', 255, 255, noop)
    cv2.createTrackbar('R2', 'Rangepicker', 255, 255, noop)

    cv2.createTrackbar('H1', 'Rangepicker', 0, 180, noop)
    cv2.createTrackbar('S1', 'Rangepicker', 0, 255, noop)
    cv2.createTrackbar('V1', 'Rangepicker', 0, 255, noop)

    cv2.createTrackbar('H2', 'Rangepicker', 180, 180, noop)
    cv2.createTrackbar('S2', 'Rangepicker', 255, 255, noop)
    cv2.createTrackbar('V2', 'Rangepicker', 255, 255, noop)

    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        sys.exit("Failed to open camera")
    while True:
        ret, img = cam.read()
        if not ret:
            sys.exit("Failed to read image")

        cv2.imshow("Rangepicker", cv2.inRange(img, *get_bgr_range()))
        key = cv2.waitKey(1)
        if key == ord('q'):
            sys.exit()
        elif key == ord('p'):
            print(tuple(color[0:3] for color in get_bgr_range()))


if __name__ == '__main__':
    main()
