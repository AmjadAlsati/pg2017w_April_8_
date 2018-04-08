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
    install_requires=['future',
                      'numpy>=1.11.2',
                      'opencv',
                      'pip>=1.5.6',
                      'Pygame>=1.9.2rc1',
                      'pysmbus>=0.1',
                      'pytest>=3.0.5',
                      'pyusb>=1.0.0',
                      'scipy>=0.18.1',
                      'setuptools>=28.8.0',
                      'tox-2.5.0',
                      'zmq',
                      ] + platform_dependent_requirements,
)
