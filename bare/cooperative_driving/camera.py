"""
This module contains different types of cameras which are accessible by a
common interface, :py:class:`CameraDevice`.
"""
from future import standard_library
standard_library.install_aliases()
import logging
from builtins import object

from .util import on_raspi

CAMERA_TYPE_PI = 'pi'
CAMERA_TYPE_OPENCV = 'opencv'

_RESOLUTION_PI = (1296,730)


def create(type=None, *args, **kwd):
    """Creates a CameraDevice.

    Supported types are 'pi' and 'opencv'. By default, a `PiCameraDevice` will be created on a Raspberry Pi
    and an `OpenCVCameraDevice` otherwise. Arguments passed to this function will be passed to the
    corresponding constructor. Refer to the corresponding docstrings for more information.
    """
    if type is None:
        type = CAMERA_TYPE_PI if on_raspi() else CAMERA_TYPE_OPENCV
    logging.info("Creating {} camera type.".format(type))

    if type == CAMERA_TYPE_PI:
        return PiCameraDevice(*args, **kwd)
    elif type == CAMERA_TYPE_OPENCV:
        return OpenCVCameraDevice(*args, **kwd)


class CameraDevice(object):
    """A CameraDevice instance represents an camera.

    This class gives access to different camera devices. Currently, SimpleCV and PiCamera devices are supported.
    """

    def close(self):
        """Closes the camera device."""

    def config(self, **kwd):
        """Configures the camera based on the supplied parameters.

        :param kwd: The values to set on the device. Supported parameters depend on the used camera. Unsupported
        parameters are ignored.

        """

    def capture(self):
        """Captures a picture and returns it.

        :returns: The captured image or None if the device is in an invalid state (e.g. not initialized, invalid type).
        :rtype: Image
        """

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return self.close()


class PiCameraDevice(CameraDevice):
    """A PiCameraDevice instance grants access to a Raspberry Pi camera."""

    def __init__(self, hflip=False, vflip=False, **kwd):
        """Creates an instance and connects to the Raspberry Pi camera."""
        try:
            import picamera
            import picamera.array
        except ImportError:
            logging.error("Cannot import picamera which is required for this camera type")
            raise
        self._device = picamera.PiCamera()
        self._output = picamera.array.PiRGBArray(self._device)
        self._device.resolution = _RESOLUTION_PI
        self._device.hflip = hflip
        self._device.vflip = vflip

    def close(self):
        self._device.close()
        self._output.close()

    def capture(self):  # FIXME test performance, change videoport and size
        self._device.capture(self._output, 'rgb', use_video_port=True)
        result = self._output.array  # copying necessary?
        self._output.truncate(0)
        return result


class OpenCVCameraDevice(CameraDevice):
    """A OpenCVCameraDevice instance grants access to cameras that are accessible by OpenCV."""

    def __init__(self, **kwd):
        try:
            import cv2
        except ImportError:
            logging.error("Cannot import cv2 which is required for this camera type")
            raise
        self._device = cv2.VideoCapture(-1)

    def close(self):
        self._device.release()

    def capture(self):
        """Captures an image.

        For now, this method silently ignores errors while taking an image.
        """
        return self._device.read()[1]
