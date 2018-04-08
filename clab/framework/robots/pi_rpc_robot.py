################
#
# Modul:        rc_simple_local_robot.py
# File:         rc_simple_robot.py
#
# Author:       Bernd Kleinjohann
# Created:      Oct. 2016
# Version:      0.1
#
# Contents:

"""
local robot emulation controled by remote procedure call
with speed and camera
"""
from builtins import str

import os
import sys
from traceback import extract_tb
from StringIO import StringIO

from framework.smart_object import SMOBaseObject, SMOSyncObject
from framework.smart_object import SMOIPID, SMO_IP_connect, SMO_IP_new_server, SMO_IP_disconnect, SMO_IP_delete_server
from framework.remote_control import rc_util
from framework.smart_node import SMONodeInterface as RCInit
from framework.devices import CameraDevice, CCSDev

ROB_CONNECTION_SAMPLE_RATE = 0.1  # rate for gui
ROB_CONTROL_SAMPLE_RATE = 0.1  # rate for control
ROB_VISION_SAMPLE_RATE = 0.1  # rate for picture grabbibg

ROB_RECONNECT_PERIOD = 1  # rate for reconnect/restart server/client

MY_TEST = True

_LISTEN_ADDRESS = '0.0.0.0'
_DEFAULT_PORT = 8008

TEST_FNAME = 'robby_file'
TEST_CONNECTION_NAME = 'robby'
TEST_ROBOT_NAME = 'robby_zero'

ROB_INPUT_PORT = {'instr': (None, rc_util.rc_token), 'setdrive': (
    [0, 0], rc_util.rc_token), 'setled': ([0, 0], rc_util.rc_value)}
ROB_OUTPUT_PORT = {'state': (None, rc_util.rc_token), 'getdrive': ([0, 0], rc_util.rc_token), 'getpos': ([0, 0, 0], rc_util.rc_value),
                   'getdistance': ([0, 0, 0, 0, 0], rc_util.rc_value), 'img': ([0, 0, None], rc_util.rc_token)}

##################################
###
# RC Controll side of connection
###


class CCSCallRobotConnectLoop(SMOSyncObject):
    """
        CCSCallRobotConnectLoop: Class spans a worker thread on GUI side for the communication
    """

    def __init__(self, robot_proxy, period=ROB_CONNECTION_SAMPLE_RATE, **kwd):
        """
            CCSCallRobotConnectLoop.__init__ Class spans a worker thread on GUI side for the communication
        """
        SMOSyncObject.__init__(self, threaded=True, auto_serialize=False,
                               auto_compute=False, period=period)
        if robot_proxy is not None:
            self.robot_proxy = robot_proxy
        else:
            if MY_TEST:
                SMOBaseObject.debug_handler.out('*** ERROR *** Get no CSCCallRobotConnect_1 instance')
        return

    def compute_handler(self, time_tab, **kwd):
        """
            CCSCallRobotConnectLoop.This compute_handler on GUI side will be called periodically
            timeta = (...) see smo_objects
        """
        #if MY_TEST:
        #    mstr = '*** INFO *** CCSCallRobotConnectLoop time table: %s' % (str(time_tab['act_cycle']))
        #    SMOBaseObject.debug_handler.out(mstr)
        self.robot_proxy.evaluate(time_tab, **kwd)
        return

class CCSCallRobotConnect_1(RCInit):
    """A stub for the robot on UI that connects to the robot."""

    def __init__(self, host='localhost', port=_DEFAULT_PORT, smo_init_mode='new', path=None, fname=TEST_FNAME,
                 connection_name=TEST_CONNECTION_NAME, robot_name=TEST_ROBOT_NAME, **kwd):
        """Instantiates the stub.

        :param host: The robot's hostname.
        :param port: The port on which to connect to the robot.

        All additional parameters are mostly unused or used for object de-/serialization.
        """
        RCInit.__init__(self, smo_init_mode=smo_init_mode,
                        input_port=ROB_INPUT_PORT, output_port=ROB_OUTPUT_PORT,
                        in_pred=None, out_succ=None)

        self.path = os.getcwd()
        if path is not None:
            self.path = path
        self.fname = fname  # file name
        self.connection_name = connection_name  # for registration as rc_node in GUI
        self.robot_name = robot_name  # robot name, should be pi hostname

        self.host = host  # pi hostname (robot)
        self.port = port  # server port of pi (robot)

        self.connection_sample_rate = ROB_CONNECTION_SAMPLE_RATE
        self.cycle = CCSCallRobotConnectLoop(self, period=self.connection_sample_rate)


        self.robot_ipid = SMOIPID(server_host=self.host, server_port=self.port,
                                  obj_id_str=self.robot_name, glob=None, smo_init_mode='new', **kwd)
        return

    def start(self):
        self.cycle.start_serialize()
        self.cycle.start_compute()
        # establish robot connection, client side
        self.client_connect = SMO_IP_connect(server_host=self.host, server_port=self.port,
                                             reconnect_period=ROB_RECONNECT_PERIOD,
                                             glob=globals())
        if MY_TEST:
            SMOBaseObject.debug_handler.out('*** INFO *** CSCCallRobotConnect_1 connected')
        return

    def stop(self):
        self.cycle.stop_compute()
        self.cycle.stop_serialize()
        if MY_TEST:
            SMOBaseObject.debug_handler.out('*** INFO *** CSCCallRobotConnect_1 diconnect')
        SMO_IP_disconnect(server_host=self.host, server_port=self.port)
        return

    def evaluate(self, timetab):
        try:
            self.move_port_data(self.init_inport, self.external_inport, self.internal_inport)
            retval = self.robot_ipid.call_communication(self.internal_inport)

            if isinstance(retval, dict):
                self.set_port_data(self.init_outport, self.internal_outport, data=retval, reset=False)
                self.move_port_data(self.init_outport, self.internal_outport, self.external_outport)
        except:
            error_list = extract_tb(sys.exc_info()[2])
            error = error_list[len(error_list) - 1]
            mstr = '*** ERROR *** RCCallRobotConnectCSCCallRobotConnect_1 evaluate robot connection %s compute_handler raises an exception:\n  %s\n File: %s , Line number: %s ,\n   Method: %s , Statement: %s'\
                % (self.connection_name, sys.exc_info()[1], error[0], error[1], error[2], error[3])
            SMOBaseObject.debug_handler.out(mstr)
            return

############################
###
# Robot side of connection
###


class CallRobotLoop(SMOSyncObject):
    """
        Class spans a worker thread on robot side for the communication
    """

    def __init__(self, call_robot, robot_name=TEST_ROBOT_NAME, period=ROB_CONTROL_SAMPLE_RATE, **kwd):

        SMOSyncObject.__init__(self, id_str=robot_name, threaded=True, auto_serialize=False,
                               auto_compute=False, period=period, **kwd)
        if call_robot is not None:
            self.call_robot = call_robot
        else:
            if self.error:
                SMOBaseObject.debug_handler.out('*** ERROR *** Get no CCSCallRobot_1 instance')
        return

    def compute_handler(self, time_tab, **kwd):
        #        if self.error:
        if self.test:
            mstr = '*** INFO *** CallRobotLoop time table: %s' % (str(time_tab['act_cycle']))
            SMOBaseObject.debug_handler.out(mstr)
        self.call_robot.evaluate(time_tab, **kwd)
        return

    def call_communication(self, inport_kwd, **kwd):
        outport_kwd = self.call_robot.call_communication(inport_kwd, **kwd)
        return outport_kwd


class CCSCallRobot_1(RCInit):
    """
    CCSCallRobot_1: Simple Robot for RASPI
    """
    glob = globals()

    def __init__(self, smo_init_mode='new', host=_LISTEN_ADDRESS, port=_DEFAULT_PORT,
                 path=None, fname=TEST_FNAME, robot_name=TEST_ROBOT_NAME,
                 cam_index=0, control_sample_rate=ROB_CONTROL_SAMPLE_RATE,
                 vision_sample_rate=ROB_VISION_SAMPLE_RATE, **kwd):
        RCInit.__init__(self, smo_init_mode=smo_init_mode,
                        input_port=ROB_INPUT_PORT, output_port=ROB_OUTPUT_PORT,
                        in_pred=None, out_succ=None)
        self.path = os.getcwd()
        if path is not None:
            self.path = path

        self.fname = fname
        self.robot_name = robot_name

        self.host = host
        self.port = port

        self.test_val = 0.5
        self.cam_index = cam_index
        self.img_sample_rate = vision_sample_rate

        self.control_sample_rate = control_sample_rate

        self.disco = 0 # disco mode status

        self.state = 'idle'
        self.cam = CameraDevice.create('pi')
        self.drive_dev = CCSDev()
        self.cycle = CallRobotLoop(self, period=self.control_sample_rate, robot_name=self.robot_name)
        return

    def start(self, **kwd):
        self.cycle.start_serialize()
        self.cycle.start_compute()
        self.server = SMO_IP_new_server(server_host=self.host, server_port=self.port, IP_prot='TCP',
                                        reconnect_period=ROB_RECONNECT_PERIOD, glob=globals(), **kwd)
        self.server.publish(self.cycle)
        return

    def stop(self):
        self.cycle.stop_compute()
        self.cycle.stop_serialize()
        SMO_IP_delete_server(self.host, self.port)
        if self.error:
            SMOBaseObject.debug_handler.out('*** INFO *** CCSCallRobot_1 stops')
        return

    def get_restore_dict(self):
        """
        This method returns a dictionary which contain all persistent data
        for all members of restore eval(repr(restore)) has to work especially for init values
        """
        if SMOBaseObject.trace:
            SMOBaseObject.debug_handler.out('*** INFO *** CCSCallRobot_1 get_restore_dict')
        ret_dict = RCInit.get_restore_dict(self)
        local_dict = {
            'rc_path': self.path,
            'rc_name': self.fname,
            'robot_name': self.robot_name,
            'cam_index': self.cam_index,
            'img_sample_rate': self.img_sample_rate,
            'control_sample_rate': self.control_sample_rate}
        ret_dict.update(local_dict)
        return ret_dict

    def call_communication(self, inport_kwd, **kwd):
        self.set_port_data(self.init_inport, self.external_inport, inport_kwd, reset=False)

        if self.test:
            SMOBaseObject.debug_handler.out('*** INFO *** CCSCallRobot_1 IN setdrive ',
                                            self.internal_inport['setdrive'])

        if self.test:
            SMOBaseObject.debug_handler.out('*** INFO *** CCSCallRobot_1 IN getpos ', self.external_outport['getpos'])

        return self.external_outport

    def evaluate(self, timetab):
        if self.test:
            SMOBaseObject.debug_handler.out('*** INFO *** CCSCallRobot_1', self.robot_name,
                                            ' evaluate at: ', timetab['act_cycle_time'])

        self.move_port_data(self.init_inport, self.external_inport, self.internal_inport)

        image = self.cam.capture()  # FIXME dirty fix of serilizing the camera image to a byte string
        str_w_file = StringIO()
        image.save(str_w_file, "JPEG")
        self.internal_outport['img'] = str_w_file.getvalue()
        str_w_file.close()

        if self.test:
            SMOBaseObject.debug_handler.out(
                '+++ eval send back: ', self.internal_outport['getdrive'], ' ', (self.internal_outport['img'] is not None))

        if self.internal_inport['instr'] is not None:
            SMOBaseObject.debug_handler.out('Instructio not none ')
            if self.internal_inport['instr'] == 'stop':
                self.state = 'stop'
                if MY_TEST:
                    SMOBaseObject.debug_handler.out('+++ eval instruction stop')
                self.stop()
            elif self.internal_inport['instr'] == 'exit':
                if MY_TEST:
                    SMOBaseObject.debug_handler.out('+++ eval instruction exit')
                self.state = 'exit'
                self.stop()
            elif self.internal_inport['instr'] == 'idle':
                self.state = 'idle'
                if MY_TEST:
                    SMOBaseObject.debug_handler.out('+++ eval instruction idle')
            elif self.internal_inport['instr'] == 'dummy':
                self.state = 'dummy'
            elif self.internal_inport['instr'] == 'led_on':
                self.state = 'led_on'
                self.drive_dev.set_led(1.0, 1.0)
            elif self.internal_inport['instr'] == 'led_off':
                self.state = 'led_off'
                self.drive_dev.set_led(0.0, 0.0)
            elif self.internal_inport['instr'] == 'disco':
                self.state = 'disco'
                if self.disco == 0:
                    self.drive_dev.set_led(0.0, 0.0)
                    self.disco = 1
                elif self.disco == 1:
                    self.drive_dev.set_led(1.0, 0.0)
                    self.disco = 2
                elif self.disco == 2:
                    self.drive_dev.set_led(0.0, 1.0)
                    self.disco = 3
                elif self.disco == 3:
                    self.drive_dev.set_led(1.0, 1.0)
                    self.disco = 0
            # extend for more instructions
            else:
                if self.test:
                    SMOBaseObject.debug_handler.out('+++ eval unknown instruction %s' %
                                                    str(self.internal_inport['instr']))

        self.internal_outport['state'] = self.state

        if self.internal_inport['setdrive'] is not None:
            steer,speed = self.internal_inport['setdrive']
	    #SMOBaseObject.debug_handler.out('Got setdrive ', speed, steer)
            self.drive_dev.set_drive(-speed, steer)
            self.internal_outport['getdrive'] = steer, speed

        self.move_port_data(self.init_outport, self.internal_outport, self.external_outport)
        return
