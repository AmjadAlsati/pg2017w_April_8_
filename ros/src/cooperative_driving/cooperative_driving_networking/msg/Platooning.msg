# Used in the demo application for the token passing protocol

# Common message header
# Standard ROS message header
Header header
# Applicationwise uniquie identifier of this message content
uint32 message_id
# The id of the robot which sends this message
uint8 sender_id
# Specific data of this message type
# TODO
uint8 platoon_id
# TODO
uint8 leader_id
# TODO
string robot_role
# TODO
uint8 predecessor_id
# TODO
string maneuver
