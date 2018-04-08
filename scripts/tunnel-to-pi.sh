#!/bin/bash

# Creates a ssh tunnel to the remote host on the configered port
# which can be used to connect to the raspberries from machines in
# the university network (e.g. machines from the rlab or the Aquarium)

# Connection representation:
# localhost:$LOCAL_PORT <--> $REMOTE_IP:$REMOTE_PORT

# You need to have ssh access configured to the gateway (i.e. ccs-aqua)
# and to the raspberries. Also you have to configure the hostnames of the
# raspberries in your ~/.ssh/config to tunnel the connection through the 
# gateway.

# The option -nNT determines to just create the port forwarding tunnel such 
# that no tty session is created on the remote host.

# For more information contact:
# Julian Heinovski <julian93@mail.upb.de>

# TODO Use command line parameters instead of static variables

# raspi 1
#REMOTE_IP="192.168.1.11"
#REMOTE_HOST="raspi1"

# raspi2
#REMOTE_IP="192.168.1.12"
#REMOTE_HOST="hasso"

# raspi3
#REMOTE_IP="192.168.1.13"
#REMOTE_HOST="bello"

REMOTE_IP="192.168.1.13"
REMOTE_HOST="bello"

REMOTE_PORT="11311"
LOCAL_PORT="11311"

echo "ssh -nNT -L $LOCAL_PORT:$REMOTE_IP:$REMOTE_PORT $REMOTE_HOST"
exec ssh -nNT -L $LOCAL_PORT:$REMOTE_IP:$REMOTE_PORT $REMOTE_HOST

# Command example after evaluation of all variables
# ssh -nNT -L 8008:192.168.1.13:8008 raspi3

# You can test the connection on the remote host by
# running 'nc -k -l $REMOTE_PORT' to listen to the port
# waiting for input. On your local machine you can send
# something through the tunnel by running
# 'cat ~/.bashrc | nc localhost 8008'
