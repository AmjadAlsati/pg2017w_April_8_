################
#
# Modul:        RC_Control
# File:         rc_init_node.py
#
# Author:       Bernd Kleinjohann
# Created:      Oct. 2015
# Version:      1.0
#
# Licence:      GNU ???
#
# Contents:     global setups and configurations for RC_Control
#

import timeit

from framework.smart_object import SMOBaseObject
from framework.smart_node import SMONodeInterface as RCInit
from . import rc_util


class RCInitTkGui(RCInit):

    # initial Tk()
    rc_tk_root = None
    # list of all toplevel windows
    rc_window_lst = []
    # dict of availabe widget for menu entry
    rc_menu_widgets = {}
    # default path for RCInitTkGui nodes
    rc_io_widgets = {}
    # default path for RCInitTkGui nodes
    rc_init_tk_path = None
    # globals for restore
    glob = None

    def __init__(self, smo_init_mode='new', rc_path=None, rc_name=None, connection_name=None, **kwd):

        self.rc_path = RCInitTkGui.rc_init_tk_path
        if rc_path is not None:
            self.rc_path = rc_path

        self.rc_name = rc_util.rc_default_model_name
        if rc_name is not None:
            self.rc_name = rc_name

        self.connection_name = rc_util.rc_default_model_name
        if connection_name is not None:
            self.connection_name = connection_name

        self.rc_top_level_widget = None
        self.rc_header_widget = None

        # window structure for restore
        self.rc_widget_structure = None

        # data for data exchange data between nodes
        self.sync_widget = None
        self.sync_run = False
        self.sync_period = rc_util.rc_control_sample_rate
        # display methods, dictionary structure { <portName>:[list of set_methods, ...], ...}
        self.sync_display_dict = {}

        RCInit.__init__(self, smo_init_mode=smo_init_mode, **kwd)

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
            'rc_widget_structure': self.rc_widget_structure,

            'rc_path': self.rc_path,
            'rc_path': self.rc_path,


            'init_inport': self.init_inport,
            'init_outport': self.init_outport,
            'out_succ': self.out_succ,
            'in_pred': self.in_pred}
        ret_dict.update(local_dict)
        return ret_dict

    def rc_sync(self):
        if self.sync_run:
            self.rc_step()
            self.sync_widget.after(self.sync_period, self.rc_sync)
        return

    def rc_step(self):
        ###
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** eval GUI ', self.connection_name, ' at: ', timeit.default_timer())

        self.init_port_tab(self.init_outport, self.internal_outport)
        # print '+++1 init out ', self.init_outport
        # print '+++2 out ', self.internal_outport

        self.get_input_fkt(self.internal_outport, self.widget_init_tab)
        # print '+++3 out', self.internal_outport

        self.move_port_data(self.init_outport, self.internal_outport, self.external_outport)
        # print '+++4 out internal ', self.internal_outport
        # print '+++5 out external ', self.external_outport

        self.distribute_port_data(source_port_tab=self.external_outport, source_init_port_tab=self.init_outport,
                                  node_tab=RCInit.node_tab, connection_tab=self.out_succ)
        #img_flag = (self.internal_inport['img']!=None)
        # print '+++6 in external ',self.external_inport['getdrive'], ' ', img_flag

        self.init_port_tab(self.init_inport, self.external_inport)
        self.collect_port_data(target_port_tab=self.external_inport, target_init_port_tab=self.init_inport,
                               node_tab=RCInit.node_tab, connection_tab=self.in_pred)
        #img_flag = (self.internal_inport['img']!=None)
        # print '+++7 in external ', self.external_inport['getdrive'], ' ', img_flag

        self.move_port_data(self.init_inport, self.external_inport, self.internal_inport)
        #img_flag = (self.internal_inport['img']!=None)
        # print '+++8 in internal ', self.internal_inport['getdrive'], ' ', img_flag

        self.set_output_fkt(self.internal_inport, self.widget_init_tab)
        return


# set display values
# for x in self.sync_display_dict and x in self.internal_inport:
##            for y in self.sync_display_dict[x]: y[self.internal_inport[x]]
##
##        self.sync_widget.after( self.sync_period, self.rc_sync)
# return
