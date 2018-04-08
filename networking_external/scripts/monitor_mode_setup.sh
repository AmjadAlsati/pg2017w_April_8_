#!/bin/bash
##### Used to create a monitoring inerface and set the appropriate listening frequency 		#####
##### phy# is the physical interface of the wifi dongle that can be checked by using `iw dev` 	#####
##### for more troubleshooting instructions see readme file 					##### 

if [ $# = "0" ]; then
        echo "Please give the phy# as argument"
        exit 1
else
set -x
sudo iw phy $1 interface add mon0 type monitor
sudo ifconfig mon0 up
sudo iw dev wlan1 del		
#sudo iw dev mon0 set freq 2412 	#iw dev sets the frequency "when possible"
sudo iwconfig mon0 channel 1		#iwconfig sets the frequency immediately
iwconfig mon0

fi

