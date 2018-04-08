################
#
# Modul:        robots.py
# File:         rc_simple_call_robot.py
#
# Author:       Bernd Kleinjohann
# Created:      Oct. 2016
# Version:      0.1
#
# Contents:
"""
local robot emulation with speed,and active camera
robot runs as own process, communication via rpc
for starting a robot call this file
"""
from builtins import str

import os
import sys
from time import sleep
from traceback import extract_tb

from framework.remote_control.rc_util import rc_token, rc_value, rc_default_model_name
from framework.devices.camera import CameraDevice
from framework.smart_node import SMONodeInterface as RCInit
from framework.smart_object import SMOBaseObject, SMOSyncObject
from framework.smart_object import (SMOIPID, SMO_IP_connect, SMO_IP_new_server,
                                         SMO_IP_disconnect, SMO_IP_delete_server)
from framework.smart_object.smo_util import smo_IP_default_reconnect_period

rob_control_sample_rate = 0.1
rob_vision_sample_rate = 0.1

rc_test = True

test_host = 'localhost'
test_port = 8008
test_connection_name = 'robby_connection'
test_robot_name = 'call_robby'

CAMERA_TYPE = 'simplecv'

##################################
###
# RC Controll side of connection
###


class CallRobotConnectLoop(SMOSyncObject):
    """A thread that calls the robot_proxy's evaluate method periodically."""

    def __init__(self, robot_proxy=None, period=rob_control_sample_rate, **kwd):
        SMOSyncObject.__init__(self, threaded=True, auto_serialize=False,
                               auto_compute=False, period=period)
        self.robot_proxy = robot_proxy
        return

    def compute_handler(self, time_tab, **kwd):
        """Called periodically by baseclass."""
        if self.test:
            mstr = '*** TEST *** CallRobotConnectLoop time table: %s' % (str(time_tab['act_cycle']))
            SMOBaseObject.debug_handler.out(mstr)
        self.robot_proxy.evaluate(time_tab, **kwd)
        return


class RCCallRobotConnect_1(RCInit):
    """The UI-Side connection.

    This is not used by the script right now.
    """

    def __init__(self, smo_init_mode='new', host=test_host, port=test_port,
                 rc_path=None, rc_name=rc_default_model_name,
                 connection_name=test_connection_name, robot_name=test_robot_name, **kwd):

        self.rc_path = os.getcwd()
        if rc_path is not None:
            self.rc_path = rc_path
        self.rc_name = rc_name  # file name
        self.connection_name = connection_name
        self.robot_name = robot_name
        self.host = host
        self.port = port

        self.cont_sample_rate = rob_control_sample_rate
        self.img_token = 3
        self.cycle = CallRobotConnectLoop(robot_proxy=self, period=self.cont_sample_rate)

        self.state = 'undef'

        # initialize interface
        input_port = {'instr': (None, rc_token), 'setdrive': ([0, 0], rc_token),
                      'setcampos': ([0, 0], rc_value), 'imgtoken': (0, rc_token)}
        output_port = {'state': (None, rc_token), 'getdrive': ([0, 0], rc_token),
                       'getcampos': ([0, 0], rc_value), 'img': ([0, 0, None], rc_token)}

        RCInit.__init__(self, smo_init_mode=smo_init_mode,
                        input_port=input_port, output_port=output_port,
                        in_pred=None, out_succ=None)

        self.robot_ipid = SMOIPID(server_host=self.host, server_port=self.port,
                                  obj_id_str=self.robot_name, glob=None, smo_init_mode='new', **kwd)
        return

    def start(self):
        """Starts the handling thread and connects to the robot."""
        self.cycle.start_serialize()
        self.cycle.start_compute()
        self.client_connect = SMO_IP_connect(server_host=self.host, server_port=self.port,
                                             reconnect_period=smo_IP_default_reconnect_period,
                                             glob=globals())
        return

    def stop(self):
        self.cycle.stop_compute()
        self.cycle.stop_serialize()
        SMO_IP_disconnect(server_host=self.host, server_port=self.port)
        return

    def evaluate(self, timetab):
        try:
            if self.test:
                SMOBaseObject.debug_handler.out('*** TEST *** RCCallRobotConnect evaluate gui side external in ',
                                                self.external_inport['setdrive'])
            self.move_port_data(self.init_inport, self.external_inport, self.internal_inport)
            retval = self.robot_ipid.call_communication(self.internal_inport)

            if isinstance(retval, dict):
                self.set_port_data(self.init_outport, self.internal_outport, data=retval, reset=False)
                self.move_port_data(self.init_outport, self.internal_outport, self.external_outport)

                if self.test:
                    img_flag = (self.external_outport['img'] is not None)
                    SMOBaseObject.debug_handler.out('*** TEST *** RCCallRobotConnect evaluate gui side internal out ',
                                                    self.external_outport['getdrive'], img_flag)
            else:
                if self.test:
                    SMOBaseObject.debug_handler.out(
                        '*** TEST *** RCCallRobotConnect evaluate gui side retval no dict ', str(type(retval)))
                pass

        except:
            if self.test:
                error_list = extract_tb(sys.exc_info()[2])
                error = error_list[len(error_list) - 1]
                SMOBaseObject.debug_handler.out('*** ERROR *** RCCallRobotConnect evaluate robot connection {} '
                                                'compute_handler raises an exception:\n  {}\n File: {} , Line '
                                                'number: {} ,\n   Method: {} , Statement: {}'.format(
                                                    self.connection_name, sys.exc_info()[1], error[0],
                                                    error[1], error[2], error[3]))
            return

############################
###
# Robot side of connection
###


class CallRobotLoop(SMOSyncObject):
    """Thread calling its call_robot's evaluate method periodically."""

    def __init__(self, call_robot=None, robot_name=test_robot_name, period=rob_control_sample_rate, **kwd):
        SMOSyncObject.__init__(self, id_str=test_robot_name, threaded=True, auto_serialize=False,
                               auto_compute=False, period=period, **kwd)
        self.call_robot = call_robot
        return

    def compute_handler(self, time_tab, **kwd):
        """Called periodically by superclass."""
        self.call_robot.evaluate(time_tab, **kwd)
        if self.test:
            SMOBaseObject.debug_handler.out('*** TEST *** CallRobot1Loop compute_handle time table:\n    {}'
                                            .format(str(time_tab)))
        return

    def call_communication(self, inport_kwd, **kwd):
        """Updates the call_robot's ports."""
        outport_kwd = self.call_robot.call_communication(inport_kwd, **kwd)
        return outport_kwd


class RCCallRobot_1(RCInit):
    """Simple robot using a camera and some engines."""
    glob = globals()

    def __init__(self, smo_init_mode='new', host=test_host, port=test_port,
                 rc_path=None, rc_name=rc_default_model_name, robot_name=test_robot_name,
                 cam_index=0, mae_id=None, cont_sample_rate=rob_control_sample_rate,
                 img_sample_rate=rob_vision_sample_rate, **kwd):
        self.rc_path = os.getcwd()
        if rc_path is not None:
            self.rc_path = rc_path

        self.rc_name = rc_name
        self.robot_name = robot_name

        self.host = host
        self.port = port

        self.test_val = 0.5
        self.cam_index = cam_index
        self.img_sample_rate = img_sample_rate

        self.mae_id = mae_id
        self.cont_sample_rate = cont_sample_rate

        self.state = 'undef'

        input_port = {'instr': (None, rc_token), 'setdrive': ([0, 0], rc_token),
                      'setcampos': ([0, 0], rc_value), 'imgtoken': (0, rc_token)}
        output_port = {'state': (None, rc_token), 'getdrive': ([0, 0], rc_token),
                       'getcampos': ([0, 0], rc_value), 'img': ([0, 0, None], rc_token)}

        RCInit.__init__(self, smo_init_mode=smo_init_mode,
                        input_port=input_port, output_port=output_port,
                        in_pred=None, out_succ=None)

        self.cam = CameraDevice.create(CAMERA_TYPE)
        self.img_token = 3
        self.dev = RCMAESimpleControl(mae_id=mae_id)
        self.cycle = CallRobotLoop(call_robot=self, period=self.cont_sample_rate, robot_name=self.robot_name)

        # Spawn server and make ports accessible
        self.server = SMO_IP_new_server(server_host=self.host, server_port=self.port, IP_prot='TCP',
                                        reconnect_period=smo_IP_default_reconnect_period, glob=globals(), **kwd)
        self.server.publish(self.cycle)
        return

    def start(self):
        """Starts the robot's operation."""
        self.cycle.start_serialize()
        self.cycle.start_compute()
        return

    def stop(self):
        """Stops the robot's operation."""
        self.cycle.stop_compute()
        self.cycle.stop_serialize()
        SMO_IP_delete_server(self.host, self.port)
        self.cam.close()
        return

    def get_restore_dict(self):
        """Returns a dict containing all persistent data."""
        ret_dict = RCInit.get_restore_dict(self)
        local_dict = {
            'rc_path': self.rc_path,
            'rc_name': self.rc_name,
            'robot_name': self.robot_name,
            'cam_index': self.cam_index,
            'img_sample_rate': self.img_sample_rate,
            'mae_id': self.mae_id,
            'cont_sample_rate': self.cont_sample_rate}
        ret_dict.update(local_dict)
        return ret_dict

    def call_communication(self, inport_kwd, **kwd):
        """Update port data."""
        self.set_port_data(self.init_inport, self.external_inport, inport_kwd, reset=False)
        return self.external_outport

    def evaluate(self, timetab):
        """Reads inports, processes instructions and sets outports."""
        if self.test:
            SMOBaseObject.debug_handler.out('*** TEST *** RCCallRobot_1 evaluate ', self.connection_name, ' at:\n    ',
                                            timetab['act_cycle_time'])

        self.move_port_data(self.init_inport, self.external_inport, self.internal_inport)

        if self.test:
            SMOBaseObject.debug_handler.out('*** TEST *** RCCallRobot_1 evaluate in ',
                                            self.internal_inport['setdrive'])

        self.internal_outport['getdrive'] = self.dev.get_move_direction()
        self.internal_outport['getcampos'] = self.dev.get_view_direction()
        self.internal_outport['img'] = self.cam.capture()

        if self.test:
            SMOBaseObject.debug_handler.out('*** TEST *** RCCallRobot_1 evaluate out ',
                                            self.internal_outport['getdrive'], ' ',
                                            (self.internal_outport['img'] is not None))

        if self.internal_inport['instr'] is not None:
            if self.internal_inport['instr'] == 'stop':
                self.state = 'stop'
                if self.test:
                    SMOBaseObject.debug_handler.out('*** TEST *** RCCallRobot_1 evaluate instruction stop')
                self.stop()
            elif self.internal_inport['instr'] == 'exit':
                self.state = 'exit'
                if self.test:
                    SMOBaseObject.debug_handler.out('*** TEST *** RCCallRobot_1 evaluate instruction exit')
                self.stop()
            elif self.internal_inport['instr'] == 'idle':
                self.state = 'idle'
                if self.test:
                    SMOBaseObject.debug_handler.out('+++ eval instruction idle')
            elif self.internal_inport['instr'] == 'dummy':
                self.state = 'dummy'
                if self.test:
                    SMOBaseObject.debug_handler.out('+++ eval instruction dummy')
            else:
                self.state = self.internal_inport['instr']
                self.internal_outport['state'] = self.state

        if self.internal_inport['setdrive'] is not None:
            speed, steer = self.internal_inport['setdrive']
            self.dev.set_move_direction(speed, steer)

        if self.internal_inport['setcampos'] is not None:
            pan, tilt = self.internal_inport['setcampos']
            self.dev.set_view_direction(pan, tilt)

        self.move_port_data(self.init_outport, self.internal_outport, self.external_outport)
        return


if __name__ == '__main__':

    SMOBaseObject.debug_handler.out('+++ main start create robot')
    rob = RCCallRobot_1(cam_index=0, host=test_host, port=test_port)

    SMOBaseObject.debug_handler.out('+++ main start robot')
    rob.start()

    s_time = 20
    SMOBaseObject.debug_handler.out('+++ main sleep for {} seconds'.format(s_time))
    sleep(s_time)
    SMOBaseObject.debug_handler.out('+++ main after sleep')

    rob.stop()
    SMOBaseObject.debug_handler.out('+++ main after robby stop')

    sleep(1)

    SMOBaseObject.debug_handler.out('+++ main end')
