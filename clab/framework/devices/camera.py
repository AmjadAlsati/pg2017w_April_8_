################
#
# Modul:        devices
# File:         camera.py
#
# Author:       Bernd Kleinjohann
# Created:      Oct. 2016
# Version:      0.1
#
# Contents:
"""
Camera package of the c-lab framework. It contains different types of cameras which are accessible by a
common interface, :py:class:`CameraDevice`.
The interface supports cameras accessible by picamera and SimpleCV.
"""
from future import standard_library
standard_library.install_aliases()
from builtins import object
import warnings
from contextlib import closing
from cStringIO import StringIO

from PIL import Image

from framework.smart_object import SMOBaseObject


def image_to_jpeg(image):
    """Returns the image encoded as a jpeg file."""
    with closing(StringIO()) as memfile:
        image.save(memfile, 'jpeg')
        return memfile.getvalue()


class CameraDevice(object):
    """A CameraDevice instance represents an camera.

    This class gives access to different camera devices. Currently, SimpleCV and PiCamera devices are supported.
    """

    @staticmethod
    def create(type, *args, **kwd):
        """Instantiates a CameraDevice.

        Supported types are 'pi', 'simplecv', 'videocapture' and 'opencv'. Arguments passed to this function will be
        passed to the corresponding constructor. Refer to the corresponding docstrings for more information.
        """
        if type == 'pi':
            return PiCameraDevice(*args, **kwd)
        elif type == 'simplecv':
            return SimpleCVCameraDevice(*args, **kwd)
        elif type == 'videocapture':
            return VideoCaptureCameraDevice(*args, **kwd)
        elif type == 'opencv':
            return OpenCVCameraDevice(*args, **kwd)

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

    def __init__(self):
        """Creates an instance and connects to the Raspberry Pi camera."""
        try:
            import picamera
        except ImportError:
            SMOBaseObject.debug_handler.out("Cannot import picamera which is required for this camera type")
            raise
        self._device = picamera.PiCamera()

    def close(self):
        self._device.close()

    def capture(self):
        img_str = StringIO()
        self._device.capture(img_str, 'jpeg', use_video_port=True, resize=(320, 240))
        img_str.seek(0)
        return Image.open(img_str)


class SimpleCVCameraDevice(CameraDevice):
    """A SimpleCVCameraDevice instance grants access to a SimpleCV camera.

    FIXME: This device is known not to work with Pillow >= 3.0.0"""

    def __init__(self):
        """Creates an instance and connects to the specified SimpleCV camera."""
        warnings.showwarning("SimpleCV is deprecated in will be dropped soon.", DeprecationWarning, "camera.py", 89)
        try:
            import SimpleCV as scv
        except ImportError:
            SMOBaseObject.debug_handler.out("Cannot import SimpleCV which is required for this camera type")
            raise
        self._device = scv.Camera()

    def capture(self):
        return self._device.getImage().getPIL()


class VideoCaptureCameraDevice(CameraDevice):
    """A VideoCaptureCameraDevice instance grants access to cameras that are accessible by VideoCapture.

    Devices accessed by this class might not work as expected after closing this instance. For more information
    see :py:meth:`close`.
    """

    def __init__(self, device_number=0):
        """Creates an instance and connects to the specified VideoCapture device.

        :param device_number: The number of the camera device to use. For more information please consult the
        VideoCapture documentation.
        """
        try:
            import VideoCapture
        except ImportError:
            SMOBaseObject.debug_handler.out("Cannot import VideoCapture which is required for this camera type")
            raise
        self._device = VideoCapture.Device(device_number)

    def close(self):
        """Closes the device.

        VideoCapture relies on the garbage collector freeing the device's memory for releasing hte device, which
        will happen after an unspecified amount of time. The camera will not be available immediatly after calling
        close.
        """
        del self._device

    def capture(self):
        return self._device.getImage()


class OpenCVCameraDevice(CameraDevice):
    """A OpenCVCameraDevice instance grants access to cameras that are accessible by OpenCV."""

    def __init__(self, device_number=0):
        try:
            import cv2
        except ImportError:
            SMOBaseObject.debug_handler.out("Cannot import cv2 which is required for this camera type")
            raise
        self._device = cv2.VideoCapture(-1)

    def close(self):
        self._device.release()

    def capture(self):
        """Captures an image.

        For now, this method silently ignores errors while taking an image.
        """
        return Image.fromarray(self._device.read()[1])
