## Command a direction change at an upcoming crossing
# Common message header
# Standard ROS message header
Header header
# Define the direction to take at the upcoming crossing
uint32 message_id
# The id of the robot which sends this message
uint8 sender_id
# Specific data of this message type

# Commnand to be executed
string command
# Parameters for the command

# Parameter for turn command
Direction turn

#TODO: Add other command parameters here