from __future__ import absolute_import
from builtins import str
################
#
# Modul:        rc_simple_local_robot.py
# File:         rc_simple_robot.py
#
# Author:       Bernd Kleinjohann
# Created:      Oct. 2015
# Version:      1.0
#
# Licence:      GNU ???
#
# Contents:     local robot emulation with speed,and active camera
#
import os
import sys
from traceback import extract_tb
from time import sleep

import framework.smart_object.smo_util as smo_util
from framework.smart_object import SMOBaseObject, SMOSyncObject, smo_set_debug_options
from framework.smart_object import SMOIPID, SMO_IP_connect, SMO_IP_new_server, SMO_IP_publish, SMO_IP_delete_all_server, SMO_IP_disconnect, SMO_IP_delete_server

from framework.remote_control import rc_util
from framework.smart_node import SMONodeInterface as RCInit
from .rc_simple_local_robot import rc_cam, rc_mae_control

rob_control_sample_rate = 0.05
rob_vision_sample_rate = 0.05

# smo_debug
rc_test = True
test_host = 'localhost'
test_port = 7000
control_test_host = 'localhost'
control_test_port = 8000
#test_id_str = 'call_robby_connection'
test_connection_name = 'robby_connection'
test_robot_name = 'send_robby'

###
# RC Controll side of connection
###


class CallRobotConnectLoop(SMOSyncObject):

    def __init__(self, robot_proxy=None, period=rob_control_sample_rate, **kwd):
        SMOSyncObject.__init__(self, threaded=True, auto_serialize=False,
                               auto_compute=False, period=period)
        self.robot_proxy = robot_proxy
        return

    def compute_handler(self, time_tab, **kwd):

        if SMOBaseObject.info:
            mstr = '*** INFO *** CallRobotConnectLoop time table: %s' % (str(time_tab['act_cycle']))
            #mstr = '*** INFO *** CallRobotConnectLoop time table: %s'%(str(time_tab))
            SMOBaseObject.debug_handler.out(mstr)
        self.robot_proxy.evaluate(time_tab, **kwd)
        return


class RCCallRobotConnect_1(RCInit):
    # verbindung im GUI prozess

    def __init__(self, smo_init_mode='new', host=test_host, port=test_port,
                 rc_path=None, rc_name=rc_util.rc_default_model_name, connection_name=test_connection_name, robot_name=test_robot_name, **kwd):

        ###
        self.rc_path = os.getcwd()
        if rc_path is not None:
            self.rc_path = rc_path
        self.rc_name = rc_name  # file name
        self.connection_name = connection_name
        self.robot_name = robot_name
        self.host = host
        self.port = port

        ### self.img_sample_rate = img_sample_rate
        self.cont_sample_rate = rob_control_sample_rate
        self.img_token = 3
        self.cycle = CallRobotConnectLoop(robot_proxy=self, period=self.cont_sample_rate)

        # definition der schnittstelle
        input_port = {'instr': (None, rc_util.rc_token), 'setdrive': ([0, 0], rc_util.rc_token), 'setcampos': (
            [0, 0], rc_util.rc_value), 'imgtoken': (0, rc_util.rc_token)}
        output_port = {'state': (None, rc_util.rc_token), 'getdrive': ([0, 0], rc_util.rc_token), 'getcampos': (
            [0, 0], rc_util.rc_value), 'img': ([0, 0, None], rc_util.rc_token)}

        RCInit.__init__(self, smo_init_mode=smo_init_mode,
                        input_port=input_port, output_port=output_port,
                        in_pred=None, out_succ=None)

# verbindung zum Roboter
# self.client_connect = SMO_IP_connect(server_host=self.host, server_port=self.port,
# reconnect_period=smo_IP_default_reconnect_period,
# glob=globals(), **kwd)

        self.robot_ipid = SMOIPID(server_host=self.host, server_port=self.port,
                                  obj_id_str=self.robot_name, glob=None, smo_init_mode='new', **kwd)

        # fuer send robot
        # self.gui_ipid =
        # self.client_connect.publish(self)

        # self.cycle.start_serialize()
        # self.cycle.start_compute()
        return

    def start(self):
        self.cycle.start_serialize()
        self.cycle.start_compute()
        # verbindung zum Roboter
        self.client_connect = SMO_IP_connect(server_host=self.host, server_port=self.port,
                                             reconnect_period=smo_util.smo_IP_default_reconnect_period,
                                             glob=globals())
        return

    def stop(self):
        self.cycle.stop_compute()
        self.cycle.stop_serialize()

        SMOBaseObject.debug_handler.out('### info befor RCCallRobotConnect_1 IP delete')

        SMO_IP_disconnect(server_host=self.host, server_port=self.port)
        return

    def evaluate(self, timetab):
        try:

            ###            SMOBaseObject.debug_handler.out('ZZZ gui side external in ', self.external_inport['setdrive'])
            self.move_port_data(self.init_inport, self.external_inport, self.internal_inport)
###            SMOBaseObject.debug_handler.out('ZZZ gui side internal in ', self.internal_inport['setdrive'])

            retval = self.robot_ipid.call_communication(self.internal_inport)

            if isinstance(retval, dict):
                ###                img_flag = self.external_outport['img']!= None
                ###                SMOBaseObject.debug_handler.out('ZZZ gui side internal out ', self.internal_outport['getdrive'], img_flag)

                self.set_port_data(self.init_outport, self.internal_outport, data=retval, reset=False)
                self.move_port_data(self.init_outport, self.internal_outport, self.external_outport)

###                img_flag = (self.external_outport['img']!=None)
###                SMOBaseObject.debug_handler.out('ZZZ gui side internal out ', self.external_outport['getdrive'], img_flag)
            else:
                ###                SMOBaseObject.debug_handler.out('ZZZ gui side retval no dict ', str(type(retval)))
                ###                self.init_port_tab(self.init_outport, self.internal_outport)
                pass
        except:
            error_list = extract_tb(sys.exc_info()[2])
            error = error_list[len(error_list) - 1]
            mstr = '*** ERROR *** RCCallRobotConnect evaluate robot connection %s compute_handler raises an exception:\n  %s\n File: %s , Line number: %s ,\n   Method: %s , Statement: %s'\
                % (self.connection_name, sys.exc_info()[1], error[0], error[1], error[2], error[3])
            SMOBaseObject.debug_handler.out(mstr)
            return

###
# Robot side of connection
###


class CallRobotLoop(SMOSyncObject):

    def __init__(self, call_robot=None, robot_name=test_robot_name, period=rob_control_sample_rate, **kwd):
        SMOSyncObject.__init__(self, id_str=test_robot_name, threaded=True, auto_serialize=False,
                               auto_compute=False, period=period, **kwd)
        self.call_robot = call_robot
        return

    def compute_handler(self, time_tab, **kwd):
        self.call_robot.evaluate(time_tab, **kwd)
        if SMOBaseObject.info:
            mstr = '*** INFO *** CallRobot1Loop compute_handle time table: %s' % (str(time_tab['act_cycle']))
            #mstr = '*** WARNING *** CallRobot1Loop compute_handle time table: %s'%(str(time_tab))
            SMOBaseObject.debug_handler.out(mstr)
        return

    #@smo_sync_method_decorator
    def call_communication(self, inport_kwd, **kwd):
        ###        SMOBaseObject.debug_handler.out('XXX rob com in ', inport_kwd['setdrive'])

        outport_kwd = self.call_robot.call_communication(inport_kwd, **kwd)

###        SMOBaseObject.debug_handler.out('XXX rob com out ', outport_kwd['getdrive'], outport_kwd['img']!=None)
        return outport_kwd


###
###
# ab hier
###
###


class RCCallRobot_1(RCInit):
    # globals for restore
    glob = globals()

    def connect_controll(self, control_host=control_test_host, control_port=control_test_port,
                         reconnect_period=smo_util.smo_IP_default_reconnect_period, glob=globals()):

        self.client_connect = SMO_IP_connect(server_host=control_host, server_port=control_port,
                                             reconnect_period=reconnect_period, glob=glob)

    def __init__(self, smo_init_mode='new', host=test_host, port=test_port,
                 rc_path=None, rc_name=rc_util.rc_default_model_name, robot_name=test_robot_name,
                 cam_index=-1, mae_id=None, cont_sample_rate=rob_control_sample_rate,
                 img_sample_rate=rob_vision_sample_rate, **kwd):

        self.rc_path = os.getcwd()
        if rc_path is not None:
            self.rc_path = rc_path

        self.rc_name = rc_name
        self.robot_name = robot_name

        self.host = host  # for robotserver
        self.port = port  # for robot server

        self.control_server_list = {}


###
        self.test_val = 0.5
        self.cam_index = cam_index
        self.img_sample_rate = img_sample_rate

        self.mae_id = mae_id
        self.cont_sample_rate = cont_sample_rate

        # {name: (token value list, init}
        input_port = {'instr': (None, rc_util.rc_token), 'setdrive': ([0, 0], rc_util.rc_token), 'setcampos': (
            [0, 0], rc_util.rc_value), 'imgtoken': (0, rc_util.rc_token)}
        output_port = {'state': (None, rc_util.rc_token), 'getdrive': ([0, 0], rc_util.rc_token), 'getcampos': (
            [0, 0], rc_util.rc_value), 'img': ([0, 0, None], rc_util.rc_token)}

        RCInit.__init__(self, smo_init_mode=smo_init_mode,
                        input_port=input_port, output_port=output_port,
                        in_pred=None, out_succ=None)

        #SMOBaseObject.debug_handler.out( 'Camera index: ', self.cam_index)

        self.cam = rc_cam(index=self.cam_index, cam_type='web_cam')
        self.img_token = 3
        self.dev = rc_mae_control(mae_id=mae_id)
        self.cycle = CallRobotLoop(call_robot=self, period=self.cont_sample_rate, robot_name=self.robot_name)

        # connecting robot, start server and publish robot object
        self.server = SMO_IP_new_server(server_host=self.host, server_port=self.port, IP_prot='TCP',
                                        reconnect_period=smo_util.smo_IP_default_reconnect_period, glob=globals(), **kwd)

        self.server.publish(self.cycle)

        # self.cycle.start_serialize()
        # self.cycle.start_compute()
        return

    def start(self):
        self.cycle.start_serialize()
        self.cycle.start_compute()
        return

    def stop(self):
        SMOBaseObject.debug_handler.out('### info befor RCCallRobot_1 stop_compute')
        self.cycle.stop_compute()
        SMOBaseObject.debug_handler.out('### info befor RCCallRobot_1 stop_serialize')
        self.cycle.stop_serialize()

        SMOBaseObject.debug_handler.out('### info befor RCCallRobot_1 IP delete')
        SMO_IP_delete_server(self.host, self.port)
        return

    def get_restore_dict(self):
        """
        This method returns a dictionary which contain all persistent data
        for all members of restore eval(repr(restore)) has to work especially for init values
        """
        if SMOBaseObject.trace:
            SMOBaseObject.debug_handler.out('*** TRACE *** PKRobotInit get_restore_dict: return {}')
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
        ###
        self.set_port_data(self.init_inport, self.external_inport, inport_kwd, reset=False)
###        SMOBaseObject.debug_handler.out('YYY rob in ', self.internal_inport['setdrive'])

###        self.move_port_data(self.init_outport, self.internal_outport, self.external_outport)
###        SMOBaseObject.debug_handler.out('YYY rob out ', self.external_outport['getdrive'], self.external_outport['img']!=None)
        return self.external_outport

    def evaluate(self, timetab):
        if SMOBaseObject.info:
            # if True:
            SMOBaseObject.debug_handler.out('*** eval simple call robot 1',
                                            self.connection_name, ' at: ', timetab['act_cycle_time'])

        self.move_port_data(self.init_inport, self.external_inport, self.internal_inport)

###
        SMOBaseObject.debug_handler.out('XXX eval in ', self.internal_inport['setdrive'])

        # input_port = {'instr':(rc_token, None), 'set_drive',(rc_token,[0,0]),
        # 'set_cam_pos':(rc_value,[0,0]), 'img_token':(rc_token,0)}
        # output_port = {'state':(rc_token, None), 'get_drive',(rc_token,[0,0]),
        # 'get_cam_pos':(rc_value,[0,0]), 'img':(rc_token,[0,0,None]) }

        self.internal_outport['getdrive'] = self.dev.get_move_direction()
        self.internal_outport['getcampos'] = self.dev.get_view_direction()
        self.internal_outport['img'] = self.cam.grab_cam()
        SMOBaseObject.debug_handler.out('XXX eval out', self.internal_outport[
                                        'getdrive'], (self.internal_outport['img'] is not None))

        if self.internal_inport['instr'] is not None:
            if self.internal_inport['instr'] == 'stop':
                if rc_test:
                    SMOBaseObject.debug_handler.out('+++ eval instruction stop')
                self.stop()
            elif self.internal_inport['instr'] == 'exit':
                if rc_test:
                    SMOBaseObject.debug_handler.out('+++ eval instruction exit')
                self.stop()
            elif self.internal_inport['instr'] == 'img_token':
                self.img_token += 1
            else:
                if rc_test:
                    SMOBaseObject.debug_handler.out('+++ eval unknown instruction %s' % self.internal_inport['instr'])

        if self.internal_inport['setdrive'] is not None:
            speed, steer = self.internal_inport['setdrive']
            self.dev.set_move_direction(speed, steer)

        if self.internal_inport['setcampos'] is not None:
            pan, tilt = self.internal_inport['setcampos']
            self.dev.set_view_direction(pan, tilt)
            #self.img_token += self.internal_inport['img_token']

        self.move_port_data(self.init_outport, self.internal_outport, self.external_outport)
###        SMOBaseObject.debug_handler.out('XXX rob com out ', self.external_outport['getdrive'], self.external_outport['img']!=None)
        return

if __name__ == '__main__':
    # start ip server
    # robot_server = SMO_IP_new_server(host=test_host, server_port=test_port, IP_prot='TCP',
    #                   reconnect_period=smo_IP_default_reconnect_period, glob=None)

    # create robot

    rob = RCCallRobot_1(cam_index=0, host=test_host, port=test_port)
    SMOBaseObject.debug_handler.out('+++ main start')
    rob.start()
    SMOBaseObject.debug_handler.out('+++ main after start')
    s_time = 30
    SMOBaseObject.debug_handler.out('+++ main sleep for %d seconds' % s_time)
    sleep(s_time)
    SMOBaseObject.debug_handler.out('+++ main after sleep')

    rob.stop()
    SMO_IP_delete_all_server()
    sys.exit()
