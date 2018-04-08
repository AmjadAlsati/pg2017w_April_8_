#!/bin/bash

#### Script is used to create a IBSS/adhoc connection on a specific frequency and assign the IP address to the	####
#### wifi device. The same script in other host with a different IP to join the IBSS/adhoc connection		####
####  --phy# is the physical device identifier of the wifi card, which can be found using `iw dev`		####

set -x
sudo ifconfig wlan1 down			#Shut(down) the interface as when its up, no changes can be made
sudo iw dev wlan1 set type ibss			#set the interface type 
sudo ifconfig wlan1 up				#Start(up) the interface
sudo iw dev wlan1 ibss join whatever 2462 	#Set the Adhoc network name and the operating frequency 
sudo iw dev wlan1 set bitrates legacy-5 12	#Set some parameters for the network at the interface level
sudo iw phy phy2 set rts off			#Set some parameters for the network at the device level
sudo ifconfig wlan1 10.10.10.11			#Provide IP address to the device. Run the same script on all other nodes with a different IP address
