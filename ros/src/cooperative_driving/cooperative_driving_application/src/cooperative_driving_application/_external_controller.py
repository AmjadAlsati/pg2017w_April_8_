"""!@package cooperative_driving_application
Defines the for the external communication class.
"""
import rospy

from cooperative_driving_common import Pubscriber
from cooperative_driving_networking.msg import CooperativeAwarenessMessage as Cam
from cooperative_driving_networking.msg import EmergencyBrake, Token, Platooning
from cooperative_driving_msgs.msg import Command
from ._message_util import create_cooperative_awareness_message
from ._message_util import create_command_message

DEFAULT_INTERVAL = 1.0

class ExternalController(object):
    """!
    External Controller class for applications.

    Provides functionality for:
        * Communicating with the external factors via Networking
        * Publishing and Receiving messages via external networking
        * Broadcasting CAMs.
    """
    def __init__(self, robot_id):

        self._cam_pub = Pubscriber('networking/cam', Cam, self.receive_network_message, 10, 10)
        self._token_pub = Pubscriber('/networking/token', Token, self.receive_network_message, 10, 10)
        self._emergency_brake_pub = Pubscriber('/networking/emergency_brake',
                                               EmergencyBrake, self.receive_network_message, 10, 10)
        self._platooning_pub = Pubscriber('/networking/platooning', Platooning, self.receive_network_message, 10, 10)
        self._command_pub = Pubscriber('/networking/command', Command, self.receive_network_message, 10, 10)

        self._beaconActive = False
        self._interval = DEFAULT_INTERVAL
        self.token_cb = None
        self.emergency_brake_cb = None
        self.cam_cb = self.handle_cam
        self.platooning_cb = None
        self.command_cb = None
        self._robot_id = robot_id

    def register_token_cb(self, handle_token):
        self.token_cb = handle_token

    def register_emergency_brake_cb(self, handle_emergency_brake):
        self.emergency_brake_cb = handle_emergency_brake

    def register_cam_cb(self, handle_cam):
        self.cam_cb = handle_cam

    def register_platooning_cb(self, handle_platooning):
        self.platooning_cb = handle_platooning

    def register_command_cb(self, handle_command):
        self.command_cb = handle_command

    def publish_message(self, message):
        """!
        Publishes a message via external networking.

        Overrides method from parent class.

        @param message: A message object which is to be sent via external networking on the token topic
        """

        if message is None:
            return

        rospy.logdebug("Transmitting network message: " + str(message))

        if isinstance(message, Token):
            self._token_pub.publish(message)
            rospy.logdebug("Transmitted token message")
        if isinstance(message, EmergencyBrake):
            self._emergency_brake_pub.publish(message)
            rospy.logdebug("Transmitted emergency brake message")
        if isinstance(message, Cam):
            self._cam_pub.publish(message)
            rospy.logdebug("Transmitted Cam message")
        if isinstance(message, Platooning):
            self._platooning_pub.publish(message)
            rospy.logdebug("Transmitted Platooning message")
        if isinstance(message, Command):
            self._command_pub.publish(message)
            rospy.logdebug("Transmitted Command message")
            

    def receive_network_message(self, message):
        """!
        Handles the reception of a message via external networking.

        Overrides method from parent class.

        @param message: A message which was received via external networking on the token topic
        """

        if message is None:
            return

        rospy.logdebug("Received network message: " + str(message))

        if isinstance(message, Token):
            rospy.logdebug("Received token message from " + str(message.sender_id))
            self.token_cb(message.sender_id, message.token_id)
        if isinstance(message, EmergencyBrake):
            rospy.logdebug("Received emergency brake message from " + str(message.sender_id))
            self.emergency_brake_cb(message.sender_id, message.enable)
        if isinstance(message, Cam):
            rospy.logdebug("Received Cam message from " + str(message.sender_id))
            self.cam_cb(message)
        if isinstance(message, Platooning):
            rospy.logdebug("Received Platooning message from " + str(message.sender_id))
            self.platooning_cb(message.sender_id, message.enable)
        if isinstance(message, Command):
            rospy.logdebug("Received DirectionCommand message from " + str(message.sender_id))
            self.command_cb(message.sender_id, message)

    def enableBeaconing(self, interval):
        """!
        Enable beacon publishing logic
        
        @param interval: Time interval to be used to configure the beacon messages
        """
        self._beaconActive = True
        self._interval = interval
        """ Implement the beaconing logic here """
        print"Enable Beaconing"
        #rospy.Timer(rospy.Duration(self._interval), self.handle_beacon_timeout, True)
        self.sendBeacon()

    def disableBeaconing(self):
        """!
        Disable beacon publishing logic
        """
        print"Disable Beaconing"
        self._beaconActive = False

    def sendBeacon(self):
        """!
        beacon send logic
        """
        if (self._beaconActive == True): 
            
            """ Make  decision to send beacon or not """
            """ send message"""
            cam = create_cooperative_awareness_message(self._robot_id, self._velocity, self._steering)
            rospy.logdebug("Publishing CAM: " + str(cam))
            self.publish_message(cam);

            rospy.Timer(rospy.Duration(self._interval), self.handle_beacon_timeout, True)


    def handle_beacon_timeout(self, event=None):
        """!
        Callback beacon interval timeout 

        @param event: An event which is forwarded from the caller according to the ROS API
        """
        self.sendBeacon()

    def handle_cam(self, message):
        """!
        Callback for handling reception of a CAM

        @param message: A received CAM which data has to be handled
        """

        rospy.loginfo("Received CAM from " + str(message.sender_id))
    
    def publish_command(self, command, parameters):
        """!
        Publish a command via network

        @param command: Command to be published
        @param parameters: Additional parameters for the command
        """

        if self._robot_id == 0:
            rospy.loginfo("Publish command" + str(command) + parameters.direction)

            message = create_command_message(self._robot_id, command, parameters)

            self.publish_message(message)
