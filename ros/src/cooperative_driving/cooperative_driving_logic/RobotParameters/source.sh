#!/bin/bash
currentDirectory=$(pwd)
File=$1
source /opt/ros/kinetic/setup.bash
echo $1 > sourced.txt
if cd $1 ; then
source setup.bash
echo "sourced: "$File "successfully!"  > error.txt
cp error.txt $currentDirectory 
rm error.txt
else
echo "source failed,No setup found or invalid directory. Try make&release then source" > error.txt
fi