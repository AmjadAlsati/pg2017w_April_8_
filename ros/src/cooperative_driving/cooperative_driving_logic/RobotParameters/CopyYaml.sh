#!/bin/bash
currentDirectory=$(pwd)
source /opt/ros/kinetic/setup.bash
sourcedPath=$(head -n 1 sourced.txt)
sourcedPath="$sourcedPath/setup.bash"
echo $sourcedPath
source $sourcedPath
roscd cooperative_driving_logic
cd Dump
cp dump.yaml $currentDirectory
