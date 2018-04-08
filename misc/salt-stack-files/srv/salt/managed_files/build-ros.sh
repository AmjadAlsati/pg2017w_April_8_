#!/bin/bash

# initialize rosdep
rosdep init
rosdep update

# create catkin workspace
mkdir -p /opt/ros/kinetic
cd /opt/ros/kinetic
rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
wstool init -j2 src kinetic-ros_comm-wet.rosinstall

# resolving dependencies
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

# building the catkin workspace
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -j2
