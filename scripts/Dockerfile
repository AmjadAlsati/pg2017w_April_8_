# start of new docker file

# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
FROM osrf/ros:kinetic-desktop-xenial

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full=1.3.1-0* libpcap-dev ros-kinetic-controller-interface ros-kinetic-controller-manager ros-kinetic-cv-camera \
&& rm -rf /var/lib/apt/lists/*
