#!/bin/bash
File=$1
File=$2
source /opt/ros/kinetic/setup.bash
rosrun dynamic_reconfigure dynparam set /reflekte $1 $2
