# Used in the demo application for the token passing protocol

# Common message header
# Standard ROS message header
Header header
# The id of the robot which sends this message
uint8 sender_id
# Specific data of this message type
# Flag to indicate whether an emergency braking situation occured or is resolved
bool enable
