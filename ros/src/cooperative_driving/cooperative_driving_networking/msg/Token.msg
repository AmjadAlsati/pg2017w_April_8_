# Used in the demo application for the token passing protocol

# Common message header
# Standard ROS message header
Header header
# Applicationwise uniquie identifier of this message content
uint32 message_id
# The id of the robot which sends this message
uint8 sender_id
# Specific data of this message type
# The token forwared by this messages represented via its id
uint32 token_id
