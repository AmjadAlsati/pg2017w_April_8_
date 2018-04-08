#!/bin/bash
File=$1
File=$2
source /opt/ros/kinetic/setup.bash
rosrun dynamic_reconfigure dynparam set /feature_extractor $1 $2
