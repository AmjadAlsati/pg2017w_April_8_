# Used in the application base class to represent a simple ETSI like CAMs

# Common message header
# Standard ROS message header
Header header
# Applicationwise uniquie identifier of this message content
uint32 message_id
# The id of the robot which sends this message
uint8 sender_id
# Specific data of this message type
# The current velocity value of this robot
float32 velocity
# The current steering value of this robot
float32 steering
# The current x coordinate of this robot
float32 posx
# The current y coordinate of this robot
float32 posy
# The current z coordinate of this robot
float32 posz
# The id of the road this robot is currently driving on
uint8 road_id
# The id of the lane this robot is currently driving on
uint8 lane_id
