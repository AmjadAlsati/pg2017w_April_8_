#!/usr/bin/python2

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from cooperative_driving_logic.srv import ChangeState, ChangeStateRequest

cmd_pub = None
change_reflekte_state = None


def joy_control_callback(msg):
    """ Callback from joy_control publisher.

    @param msg Message that arrived on the joy topic.
    """
    button_map = {
        0: ChangeStateRequest.IDLE,
        1: ChangeStateRequest.FOLLOW_LINE,
        2: ChangeStateRequest.PLATOONING,
        3: ChangeStateRequest.FOLLOW_BLOB,
        4: ChangeStateRequest.DYNAMIC_FOLLOW_LINE,
        5: ChangeStateRequest.REMOTE_CONTROL
    }

    twist_msg = Twist()
    twist_msg.linear.x = msg.axes[1]
    twist_msg.angular.z = -1 * msg.axes[2]
    cmd_pub.publish(twist_msg)
    if 1 in msg.buttons and msg.buttons.index(1) in button_map:
        change_reflekte_state(button_map[msg.buttons.index(1)])
    return True


def main():
    """
    A simple node that translates gamepad input to commands.
    """
    global cmd_pub, change_reflekte_state

    rospy.init_node('remote_control', anonymous=True)

    rospy.Subscriber('/joy', Joy, joy_control_callback, queue_size=1, buff_size=2 ** 20)
    cmd_pub = rospy.Publisher('/remote_control/cmd_vel', Twist, queue_size=1)
    change_reflekte_state = rospy.ServiceProxy('/reflekte/change_state', ChangeState)

    rospy.spin()


if __name__ == '__main__':
    main()
