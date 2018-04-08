#!/bin/bash

sudo -n true
if [ $? -eq 0 ] 
then
	if [ -f ${1} ]
	then
		sudo setcap cap_net_raw=eip ${1}
	else
		echo "ERROR: File ${1} does not exist."
	fi
else
	echo "You are not a sudoer!"
fi

