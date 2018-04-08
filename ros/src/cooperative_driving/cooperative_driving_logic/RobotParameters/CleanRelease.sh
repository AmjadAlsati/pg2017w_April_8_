#!/bin/bash
source /opt/ros/kinetic/setup.bash
currentDirectory=$(pwd)
File=$1
if cd $1 ; then
echo " " > error.txt 
make clean >> error.txt
make release >> error.txt
cp error.txt $currentDirectory 
rm error.txt
else
echo "make clean&release failed,No target found or an invalid directory" > error.txt
fi
