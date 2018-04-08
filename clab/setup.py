import platform
from setuptools import setup, find_packages

if platform.machine() == 'armv7l':  # Raspberry pi
    platform_dependent_requirements = ['picamera>=1.12']
else:
    platform_dependent_requirements = []


setup(
    name="clab_framework",
    description="Framework for running and debugging robots.",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        'pip>=1.5.6',
        'setuptools>=28.8.0',
        'future',
        'SimpleCV==1.3',
        'Pygame>=1.9.2rc1',
        'Pillow<3.0.0',  # release 3.0.0 breaks SimpleCV.Image.getPIL, we need to drop SimpleCV or revive it.
        'pyusb>=1.0.0',
        'scipy>=0.18.1',
        'numpy>=1.11.2',
        'pysmbus>=0.1',
        'pytest>=3.0.5',
        'tox-2.5.0'
    ] + platform_dependent_requirements,
)
