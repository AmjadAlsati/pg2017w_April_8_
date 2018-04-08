################
#
# Modul:        robots
# File:         rc_simple_local_robot.py
#
# Author:       Bernd Kleinjohann
# Created:      Oct. 2016
# Version:      0.1
#
# Contents:
"""
local robot emulation with speed,and active camera
robot span an own thread in rc_control, perform all data conversions
but communicate locally in the same process
mainly for test and development purpose
"""
from builtins import str
import os
import sys
from time import sleep

from framework.remote_control import rc_util
from framework.devices.camera import CameraDevice
from framework.smart_node import SMONodeInterface as RCInit
from framework.smart_object import SMOBaseObject, SMOSyncObject

rob_control_sample_rate = 0.1
rob_vision_sample_rate = 0.1

# smo_debug
rc_test = True

CAMERA_TYPE = 'simplecv'


class rc_loop(SMOSyncObject):

    def __init__(self, robot=None, period=rob_control_sample_rate, **kwd):
        SMOSyncObject.__init__(self, threaded=True, auto_serialize=False,
                               auto_compute=False, period=period, **kwd)
        self.robot = robot
        return

    def compute_handler(self, time_tab, **kwd):
        self.robot.evaluate(time_tab, **kwd)
# if SMOBaseObject.warning:
##            mstr = '*** WARNING *** call compute_handle time table: %s'%(str(time_tab))
##            SMOBaseObject.debug_handler.out( mstr )
        return


class RCLocalRobot_1(RCInit):
    # globals for restore
    glob = globals()

    def __init__(self, smo_init_mode='new',
                 rc_path=None, rc_name=None, connection_name=None,
                 cam_index=0, mae_id=None,
                 cont_sample_rate=rob_control_sample_rate,
                 img_sample_rate=rob_vision_sample_rate, **kwd):

        self.rc_path = os.getcwd()
        if rc_path is not None:
            self.rc_path = rc_path
        self.rc_name = rc_util.rc_default_model_name
        if rc_name is not None:
            self.rc_name = rc_name
        self.connection_name = rc_util.rc_default_model_name
        if connection_name is not None:
            self.connection_name = connection_name

        self.cam_index = cam_index
        self.img_sample_rate = img_sample_rate

        self.mae_id = mae_id
        self.cont_sample_rate = cont_sample_rate

        self.state = 'undef'

        # {name: (token value list, init}
        input_port = {'instr': (None, rc_util.rc_token), 'setdrive': ([0, 0], rc_util.rc_token), 'setcampos': (
            [0, 0], rc_util.rc_value), 'imgtoken': (0, rc_util.rc_token)}
        output_port = {'state': (None, rc_util.rc_token), 'getdrive': ([0, 0], rc_util.rc_token), 'getcampos': (
            [0, 0], rc_util.rc_value), 'img': ([0, 0, None], rc_util.rc_token)}

        RCInit.__init__(self, smo_init_mode=smo_init_mode,
                        input_port=input_port, output_port=output_port,
                        in_pred=None, out_succ=None)

        #SMOBaseObject.debug_handler.out( 'Camera index: ', self.cam_index)

        self.cam = CameraDevice.create(CAMERA_TYPE)
        self.dev = RCMAESimpleControl(mae_id=mae_id)
        self.cycle = rc_loop(robot=self, period=self.cont_sample_rate)
        return

    def start(self):
        self.cycle.start_serialize()
        self.cycle.start_compute()
        return

    def stop(self):
        self.cycle.stop_compute()
        self.cycle.stop_serialize()
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
            'connection_name': self.connection_name,
            'cam_index': self.cam_index,
            'img_sample_rate': self.img_sample_rate,
            'mae_id': self.mae_id,
            'cont_sample_rate': self.cont_sample_rate}
        ret_dict.update(local_dict)
        return ret_dict

    def evaluate(self, timetab):

        ###
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** eval simple local robot ',
                                            self.connection_name, ' at: ', timetab['act_cycle_time'])

# if self.error:
        if self.test:
            SMOBaseObject.debug_handler.out(
                '*** TEST *** RCLocalRobot evaluate  external in ', self.external_inport['setdrive'])

        self.move_port_data(self.init_inport, self.external_inport, self.internal_inport)

        # input_port = {'instr':(rc_token, None), 'set_drive',(rc_token,[0,0]),
        # 'set_cam_pos':(rc_value,[0,0]), 'img_token':(rc_token,0)}
        # output_port = {'state':(rc_token, None), 'get_drive',(rc_token,[0,0]),
        # 'get_cam_pos':(rc_value,[0,0]), 'img':(rc_tokem,[0,0,None]) }

        self.internal_outport['getdrive'] = self.dev.get_move_direction()
        self.internal_outport['getcampos'] = self.dev.get_view_direction()
        self.internal_outport['img'] = self.cam.grab_cam()

        if self.internal_inport['instr'] is not None:
            if self.internal_inport['instr'] == 'stop':
                if rc_test:
                    SMOBaseObject.debug_handler.out('+++ eval instruction stop')
                self.cycle.stop_compute()
            elif self.internal_inport['instr'] == 'exit':
                if rc_test:
                    SMOBaseObject.debug_handler.out('+++ eval instruction exit')
                self.cycle.stop_compute()
                self.cycle.stop_serialize(thr_exit=True)
            elif self.internal_inport['instr'] == 'img_token':
                self.img_token += 1
            else:
                self.state = self.internal_inport['instr']
                self.internal_outport['state'] = self.state
                #if rc_test: SMOBaseObject.debug_handler.out('+++ eval unknown instruction %s'%self.internal_inport['instr'])

        if self.internal_inport['setdrive'] is not None:
            speed, steer = self.internal_inport['setdrive']
            self.dev.set_move_direction(speed, steer)

        if self.internal_inport['setcampos'] is not None:
            pan, tilt = self.internal_inport['setcampos']
            self.dev.set_view_direction(pan, tilt)
            #self.img_token += self.internal_inport['img_token']

        self.move_port_data(self.init_outport, self.internal_outport, self.external_outport)

# if self.error:
        if self.test:
            img_flag = (self.external_outport['img'] is not None)
            SMOBaseObject.debug_handler.out('*** TEST *** RCLocalRobot evaluate external out ',
                                            self.external_outport['getdrive'], ' ', img_flag)

        return

if __name__ == '__main__':
    SMOBaseObject.debug_handler.out('+++++++++++++++++++++++++++++++++++')
    SMOBaseObject.debug_handler.out('+++ Start testsequence for RCLocalRobot_1')
    SMOBaseObject.debug_handler.out('+++++++++++++++++++++++++++++++++++')

    rob = RCLocalRobot_1()
    rob.start()

    s_time = 2
    SMOBaseObject.debug_handler.out('+++ main sleep for %d seconds' % s_time)
    sleep(s_time)

    ### input_port = {'instr':(None, rc_token), 'set_drive':([0,0], rc_token), 'set_cam_pos':([0,0], rc_value), 'img_token':(0, rc_token)}
    ### output_port = {'state':(None, rc_token), 'get_drive':([0,0], rc_token), 'get_cam_pos':([0,0], rc_value), 'img':([0,0,None], rc_token) }

    rob.external_inport['setdrive'] = [-1, -1]
    rob.external_inport['setcampos'] = [-1, -1]
    SMOBaseObject.debug_handler.out('+++ drive: %s' % (str(rob.external_outport['getdrive'])))
    SMOBaseObject.debug_handler.out('+++ main  (-1) sleep1 for %d seconds' % s_time)
    sleep(s_time)
    SMOBaseObject.debug_handler.out('+++++++++++++++++++++++++++++++++++')

    rob.external_inport['setdrive'] = [-0.5, -0.5]
    rob.external_inport['setcampos'] = [1, 1]
    SMOBaseObject.debug_handler.out('+++ drive: %s' % (str(rob.external_outport['getdrive'])))
    SMOBaseObject.debug_handler.out('+++ main  (-0.5) sleep1 for %d seconds' % s_time)
    sleep(s_time)
    SMOBaseObject.debug_handler.out('+++++++++++++++++++++++++++++++++++')

    rob.external_inport['setdrive'] = [0, 0]
    rob.external_inport['setcampos'] = [-1, -1]
    SMOBaseObject.debug_handler.out('+++ drive: %s' % (str(rob.external_outport['getdrive'])))
    SMOBaseObject.debug_handler.out('+++ main  (0) sleep1 for %d seconds' % s_time)
    sleep(s_time)
    SMOBaseObject.debug_handler.out('+++++++++++++++++++++++++++++++++++')

    rob.external_inport['setdrive'] = [0.5, 0.5]
    rob.external_inport['setcampos'] = [1, 1]
    SMOBaseObject.debug_handler.out('+++ drive: %s' % (str(rob.external_outport['getdrive'])))
    SMOBaseObject.debug_handler.out('+++ main 0.5 sleep1 for %d seconds' % s_time)
    sleep(s_time)
    SMOBaseObject.debug_handler.out('+++ drive: %s' % (str(rob.external_outport['getdrive'])))

    # sleep(240)

    rob.external_inport['setdrive'] = [1, 1]
    rob.external_inport['setcampos'] = [-1, -1]
    SMOBaseObject.debug_handler.out('+++ drive: %s' % (str(rob.external_outport['getdrive'])))
    SMOBaseObject.debug_handler.out('+++ main 0.5 sleep1 for %d seconds' % s_time)
    sleep(s_time)
    SMOBaseObject.debug_handler.out('+++ drive: %s' % (str(rob.external_outport['getdrive'])))

    SMOBaseObject.debug_handler.out('+++++++++++++++++++++++++++++++++++')
    rob.external_inport['instr'] = 'exit'
    SMOBaseObject.debug_handler.out('+++ End of test')
    SMOBaseObject.debug_handler.out('+++++++++++++++++++++++++++++++++++')

    # if exit does not work
    # rob.stop()
    sys.exit()
