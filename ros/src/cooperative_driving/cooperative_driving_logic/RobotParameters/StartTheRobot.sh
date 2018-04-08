#!/bin/bash
ID=$1
Mode=$2
source /opt/ros/kinetic/setup.bash
sourcedPath=$(head -n 1 sourced.txt)
sourcedPath="$sourcedPath/setup.bash"
source $sourcedPath
if [ $Mode -eq 1 ]; then
roscd cooperative_driving_logic
cd Dump
rm dump.yaml
fi
roslaunch cooperative_driving demo_raspi.launch id:=$1 > error.txt
