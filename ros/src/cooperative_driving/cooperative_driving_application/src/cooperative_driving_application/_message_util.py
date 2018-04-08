"""!@package cooperative_driving_application
Provides functionality to create messages to be used by applications.
"""

# from builtins import str
from builtins import object

import rospy

from cooperative_driving_networking import msg
from cooperative_driving_msgs import msg as commonmsg


class PlatoonManeuverTypes(object):
    """!
    Enum for Platoon maneuver types

    @deprecated: Simple protoyping approach
    """

    ## Identifier for idle maneuever
    IDLE = 0
    ## Identifier for the join maneuver request
    JOIN = 1
    ## Identifier for the leave maneuver request
    LEAVE = 2
    ## Identifier for a stop maneuver
    STOP = 3
    ## Identifier for revoking the stop maneuver state
    GO = 4


def fill_common_message_fields(message, sender_id):
    """!
    Encapsultes filling of common fields of messages

    @param message: The message object which has to be filled
    @param sender_id: The id of the robot which is sending this message
    """

    message.header.stamp = rospy.Time.now()
    message.sender_id = sender_id

    return message


def create_token_message(robot_id, token_id):
    """!
    Creates a message for the token protocol

    @param robot_id: The id of the robot which is sending this message
    @param token_id: The id of the token which this robots is currently forwarding
    """

    message = msg.Token()

    message.token_id = token_id

    message = fill_common_message_fields(message, robot_id)
    rospy.logdebug("Created token message")

    return message


def create_emergency_braking_message(robot_id, enable=True):
    """!
    Creates a message for emergency braking

    @param robot_id: The id of the robot which is sending this message
    @param enable: The status of the emergency braking. True, if this robot currently is braking. False, else.
    """

    message = msg.EmergencyBrake()

    message.enable = enable

    message = fill_common_message_fields(message, robot_id)
    rospy.logdebug("Created emergency braking message")

    return message


def create_platooning_message(
        robot_id,
        platoon_id=None,
        leader_id=None,
        robot_role=None,
        predecessor_id=None,
        maneuver=PlatoonManeuverTypes.IDLE):
    """!
    Creates a message for platooning

    @param robot_id: The id of the robot which is sending this message
    @param platoon_id: The current id of the platoon this robot belongs to
    @param leader_id: The leader id of the current platoon the robot belongs to
    @param robot_role: The current role of this robot
    @param predecessor_id: The current id of the predecessor of this robot in the platoon
    @param maneuver: The maneuver type of this message
    """

    message = msg.Platooning()

    message.platoon_id = platoon_id
    message.leader_id = leader_id
    message.robot_role = robot_role
    message.predecessor_id = predecessor_id
    message.maneuver = maneuver

    message = fill_common_message_fields(message, robot_id)
    rospy.logdebug("Created platooning message")

    return message


def create_cooperative_awareness_message(robot_id, velocity, steering, posx=0,
                                         posy=0, posz=0, road_id=1, lane_id=1):
    """!
    Creates a message for cooperative awareness

    @param robot_id: The id of the robot which is sending this message
    @param velocity: The current velocity value of this robot
    @param steering: The current steering value of this robot
    @param posx: The current x position of this robot in a cartesion coordinate system
    @param posy: The current y position of this robot in a cartesian coordinate system
    @param posz: The current z position of this robot in a cartesion coordinate system
    @param road_id: The current road id this robot is driving on
    @param lane_id: The current lane id this robot is driving on
    """

    message = msg.CooperativeAwarenessMessage()

    message.velocity = velocity
    message.steering = steering
    message.posx = posx
    message.posy = posy
    message.posz = posz
    message.road_id = road_id
    message.lane_id = lane_id

    message = fill_common_message_fields(message, robot_id)
    rospy.logdebug("Created cooperative awareness message")

    return message

def create_command_message(robot_id, command, parameter):
    """!
    Creates a message for sending command

    @param robot_id: The id of the robot which is sending this message
    @param command: The command to sent
    @param parameter: Additional parameter provided for the command
    """

    message = commonmsg.Command()

    message.command = command
    if command == "Turn":
        message.turn = parameter

    message = fill_common_message_fields(message, robot_id)
    rospy.logdebug("Created command message")

    return message