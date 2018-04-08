################
#
# Modul:        smart_node
# File:         smo_node_interface
#
# Author:       Bernd Kleinjohann
# Created:      Oct. 2016
# Version:      0.1
#
# Contents:
"""
This file contain an interface class for smart nodes
this module should be extended by

"""
from builtins import str
import copy

from framework.smart_object import SMOBaseObject
from .smo_node_util import smo_value, smo_token, smo_list


class SMONodeInterface(SMOBaseObject):
    """Class for a smad node interface TODO"""

    # global class definitions for
    glob = None

    # Dictionary of subnodes, entries {<subnodename>: <RCInit reference>}"""
    node_tab = {}

    def __init__(self, smo_init_mode='new',
                 input_port=None, output_port=None,
                 in_pred=None, out_succ=None, **kwd):
        SMOBaseObject.__init__(self, smo_init_mode=smo_init_mode, **kwd)

# file name and path
        if smo_init_mode == 'new':
            # Input port dicts
            self.init_inport = {}
            self.internal_inport = {}
            self.external_inport = {}
            # Connections from output ports of to external Input ports for pulling data
            self.init_in_pred = {}
            self.in_pred = {}

            # Output port dicts
            self.init_outport = {}
            self.internal_outport = {}
            self.external_outport = {}
            # Connections from output ports of to external Input ports for pulling data
            self.init_out_succ = {}
            self.init_out_succ = {}
            self.out_succ = {}

            # Initialisation of the input output and connection tables
            if input_port is not None:
                self.update_port_tab(input_port, self.init_inport, self.internal_inport, self.external_inport)
            if output_port is not None:
                self.update_port_tab(output_port, self.init_outport, self.internal_outport, self.external_outport)
            if in_pred is not None:
                self.update_connection_tab(in_pred, self.init_in_pred, self.in_pred, reset=False)
            if out_succ is not None:
                self.update_connection_tab(out_pred, self.init_out_succ, self.out_succ, reset=False)
        return

    def get_restore_dict(self):
        """
        This method returns a dictionary which contain all persistent data
        for all members of restore eval(repr(restore)) has to work especially for init values
        """
        if SMOBaseObject.trace:
            SMOBaseObject.debug_handler.out('*** TRACE *** PKRobotInit get_restore_dict: return {}')
        ret_dict = SMOBaseObject.get_restore_dict(self)
        local_dict = {
            'init_inport': self.init_inport,
            'internal_inport': self.internal_inport,
            'external_inport': self.external_inport,
            'init_outport': self.init_outport,
            'internal_outport': self.internal_outport,
            'external_outport': self.external_outport,
            'init_out_succ': self.init_out_succ,
            'out_succ': self.out_succ,
            'init_in_pred': self.init_in_pred,
            'in_pred': self.in_pred}
        ret_dict.update(local_dict)
        return ret_dict

# Methods for port table handling

    def update_port_tab(self, para_tab, init_tab, internal_tab, external_tab, reset=False):
        # This method constructs/Updates the port table.
        # If reset the existing ports and buffers were deleted,
        # otherwise the ports of ParTab were added to the Port dict.
        # Existing port names were replaced by new once.
        # para_tab has the form { <portname>: (init_value,port_type), ...} where port_type is smo_value, smo_token, smo_list
        # internal_tab, and external_tab has the form {portname: value(iff smo_value, smo_token) or portname:[value](iff smo_list),...}
        # init_tab has the form {portname: (init_value,port_type),...}

        if reset:
            init_tab.clear()
            internal_tab.clear()
            external_tab.clear()
        if para_tab is None:
            return

        for x in para_tab:
            # For every port specification
            port_init_value, port_type = para_tab[x]
            if port_type == smo_list:
                # Create buffer for 'append' type. The buffers are lists.
                if port_init_value is None:
                    port_init_value = []
                if not isinstance(port_init_value, list):
                    port_init_value = [port_init_value]
                init_tab[x] = (port_init_value, port_type)
                internal_tab[x] = []
                external_tab[x] = []
            elif port_type == smo_token:
                # Create buffer for 'replace' type. The buffers are values which will be overwritten.
                init_tab[x] = (port_init_value, port_type)
                internal_tab[x] = None
                external_tab[x] = None
            elif port_type == smo_value:
                # Create buffer for 'replace' type. The buffers are values which will be overwritten.
                init_tab[x] = (port_init_value, port_type)
                internal_tab[x] = copy.deepcopy(port_init_value)
                external_tab[x] = copy.deepcopy(port_init_value)
            else:
                if SMOBaseObject.warning:
                    SMOBaseObject.debug_handler.out(
                        '*** warning *** RCInit update_port_tab: Port %s illegal Input Type %s'
                        % (x, str(port_type)))
        return

    def reset_port_tab(self, init_tab, tab):
        if tab is None:
            tab = {}
        if init_tab is None:
            return
        for x in init_tab:
            if x in tab:
                if init_tab[x][1] == smo_list:
                    tab[x] = []
                elif init_tab[x][1] == smo_value:
                    tab[x] = copy.deepcopy(init_tab[x][0])
                else:
                    tab[x] = None  # init_tab[x][1] == smo_token
            else:
                if SMOBaseObject.warning:
                    SMOBaseObject.debug_handler.out(
                        '*** WARNING *** RCInit reset_port_tab: can not find port %s' % x)
        return tab

    def init_port_tab(self, init_tab, tab):
        # This not synchronized method initialize the values of a buffer according to the values in init_tab"""
        if tab is None or init_tab is None:
            return
        for x in tab:
            if x in init_tab:
                # append
                if init_tab[x][1] == smo_list:
                    if init_tab[x][0] is None:
                        tab[x] = []
                        init_tab[x][0] = []
                    # should not occure if init_tab is consistent
                    elif not isinstance(init_tab[x][0], list):
                        if SMOBaseObject.warning:
                            SMOBaseObject.debug_handler.out(
                                '*** WARNING *** RCInit init_port_tab: Port: %s repaire init_tab (create [])' % x)
                        tab[x] = [init_tab[x][0]]
                        # repare init_tab
                        init_tab[x][0] = [copy.deepcopy(init_tab[x][0])]
                    else:
                        tab[x] = []
                        tab[x].extend(copy.deepcopy(init_tab[x][0]))
                else:
                    tab[x] = init_tab[x][0]  # init_tab[x][1] == smo_value or init_tab[x][1] == smo_token:
            else:
                if SMOBaseObject.warning:
                    SMOBaseObject.debug_handler.out(
                        '*** WARNING *** RCInit init_port_tab: can not find and init port %s ' % x)
        return

    def update_port_init_data(self, init_data, init_tab, reset=False):
        if reset:
            for x in init_tab:
                if init_tab[x][1] == smo_list:
                    init_tab[x][0] = []
                else:
                    init_tab[x][0] = None
        for x in init_data:
            if x in init_tab:
                # append
                if init_tab[x][1] == smo_list:
                    if init_data[x] is None:
                        init_tab[x][0] = []
                    elif not isinstance(init_data[x], list):
                        init_tab[x][0] = [init_data[x]]
                    else:
                        ini_tab[x][0] = []
                        ini_tab[x][0].extend(init_data[x])
                # replace
                else:
                    # smo_value smo_token
                    init_tab[x][0] = init_data[x]
            else:
                if SMOBaseObject.warning:
                    SMOBaseObject.debug_handler.out(
                        '*** WARNING *** RCInit  update_port_init_data: can not find port %s ' % x)
        return

# def set_input(self, data=None, reset=False):
# self.parent.com_lock.acquire(blocking=1)
##        self.set_port_data(self.init_inport, self.inport, data=data, reset=reset)
# self.parent.com_lock.release()
# return

    def set_port_data(self, init_tab, tab, data=None, reset=False):
        # The selection is done by the keywords which are included in data {<portname>:value, ...},
        if reset:
            for x in init_tab:
                if init_tab[x][1] == smo_list:
                    tab[x] = []
                elif init_tab[x][1] == smo_value:
                    tab[x] = init_tab[x][0]
                else:
                    tab[x] = None
        if data is None:
            return
        for x in data:
            if x in tab:
                if init_tab[x][1] == smo_list:
                    if tab[x] is None:
                        tab[x] = []
                    if data[x] is not None:
                        if isinstance(data[x], list):
                            tab[x].extend(data[x])
                        else:
                            tab[x].append(data[x])
                elif init_tab[x][1] == smo_value:
                    if data[x] is not None:
                        tab[x] = data[x]
                else:
                    tab[x] = data[x]  # smo_token
            else:
                if SMOBaseObject.warning:
                    SMOBaseObject.debug_handler.out(
                        '*** WARNING *** RCInit set_port_data: can not find port %s ' % x)
        return

# def get_output(self, data=None):
# self.parent.com_lock.acquire(blocking=1)
##        self.get_port_data( self.init_outport, self.outport, data=data)
# self.parent.com_lock.release()
# return data

    def get_port_data(self, init_tab, tab, data=None):
        # returns selected values from a buffer
        # The selection is done by the keywords which are included in data data=None returns all values

        if data is None:
            data = {}
        if data == {}:
            for x in tab:
                if init_tab[x][1] == smo_list:
                    data[x] = []
                    data[x].extend(tab[x])
                else:
                    data[x] = tab[x]  # smo_value, smo_token
            return data
        for x in data:
            if x in tab:
                if init_tab[x][1] == smo_list:
                    # may extend an existing list
                    if not isinstance(data[x], list):
                        data[x] = []
                    data[x].extend(tab[x])
                else:
                    data[x] = tab[x]  # smo_value, smo_token
        return data

    def copy_port_data(self, init_tab, source_tab, target_tab):
        # This method copies the value from an source in/outport to a target in/outport
        # source is not modified, target values appended"""

        for x in source_tab:
            if x not in init_tab:
                if SMOBaseObject.error:
                    SMOBaseObject.debug_handler.out(
                        '*** ERROR *** RCInit copy_port_data: could not find port %s in init_tab' % x)
            elif x not in target_tab:
                if SMOBaseObject.warning:
                    SMOBaseObject.debug_handler.out(
                        '*** ERROR *** RCInit copy_port_data: could not find port in tab  %s' % x)
            elif init_tab[x][1] == smo_list:
                if isinstance(source_tab[x], list):
                    if target_tab[x] is None:
                        target_tab[x] = []
                        target_tab[x].extend(source_tab[x])
                    elif isinstance(target_tab[x], list):
                        target_tab[x].extend(source_tab[x])
                    else:
                        if SMOBaseObject.warning:
                            SMOBaseObject.debug_handler.out(
                                '*** ERROR *** RCInit copy_port_data: illegal data (no list or None) in target_tab port %s' % x)
                elif source_tab[x] is not None:
                    if SMOBaseObject.warning:
                        SMOBaseObject.debug_handler.out(
                            '*** ERROR *** RCInit copy_port_data: illegal data (no list or None) in source_tab port %s' % x)
                else:
                    pass
            else:
                target_tab[x] = source_tab[x]  # smo_value, smo_token
        return

    def move_port_data(self, init_tab, source_tab, target_tab):
        # This method copies the value from an source input port to a target input port
        # old values in target were overwritten, source will be resetted"""
        target_tab.clear()
        target_tab.update(source_tab)
        # source_tab.clear()
        self.reset_port_tab(init_tab, source_tab)
        return

# connection tab

    def update_connection_tab(self, para_tab, init_tab, result_tab, reset=False):
        # This method constructs the rename tables for port connections.
        # If reset the existing rename tables were deleted, otherwise the
        # connections were added. Existing Entries were replace by new once
        # para_tab={<nodename>:{<portname1(source)>:<portname2(destination)>}, ...}
        # edge source->destination mirrow_tab destination->source
        # result_tab={<nodename>:({<portname1>:<portname2>,
        # ...},{<portname2>:<portname1>, ...},{<portname1>:None,
        # ...},{<portname2>:None, ...})}

        if reset:
            result_tab.clear()
            init_tab.clear()
        if para_tab is None:
            return

        for x in para_tab:
            # For every connected node  create rename tab for direction inport to outport
            edge = para_tab[x].copy()
            # Create a temporary buffer for collect/distribut methods
            inpara = edge.copy()
            for y in inpara:
                inpara[y] = None
            mirrow_edge = {}
            # Create a rename table for direction outport to inport (mirrow)
            for y in edge:
                if edge[y] in mirrow_edge:
                    # Two inport were connected to the same outport
                    if SMOBaseObject.error:
                        SMOBaseObject.debug_handler.out(
                            '*** ERROR *** RCInit update_connection_tab: connected node %s: Portname %s not unique' % (x, y))
                else:
                    # update mirrow_tab
                    mirrow_edge[edge[y]] = y
                    # update init_tab
                    if x in init_tab:
                        init_tab[x][y] = edge[y]
                    else:
                        init_tab[x] = {y: edge[y]}
            # Create a temporary buffer for collect/distribute methods
            outpara = mirrow_edge.copy()
            for y in outpara:
                outpara[y] = None
            # set result tab for node x
            result_tab[x] = (edge, mirrow_edge, inpara, outpara)
        return

    def distribute_port_data(self, source_port_tab=None, source_init_port_tab=None,
                             node_tab=None, connection_tab=None):
        # Send values of source_port_tab to the connected Input_ports of nodes from node_tab

        if node_tab is None or source_port_tab is None or source_init_port_tab is None or connection_tab is None:
            if SMOBaseObject.warning:
                SMOBaseObject.debug_handler.out(
                    '*** WARNING *** RCInit  distribute_port_data: source for distribution not defined (act_inport,outport)')
            return

        for x in connection_tab:
            if x not in node_tab:  # target node
                if SMOBaseObject.info:
                    SMOBaseObject.debug_handler.out(
                        '*** WARNING *** RCInit  distribute_port_data: can not find successor node %s' % x)
                continue
            # hierarchie und trigger wird nicht beruecksichtigt !!!
            target_node = node_tab[x]
            target_tab = node_tab[x].external_inport
            target_init_tab = node_tab[x].init_inport

            mirrow_edge = connection_tab[x][1]
            outpara = connection_tab[x][3]

            # copy source/port to destinatin/port
            for y in outpara:  # target portname
                if not (y in mirrow_edge):  # info
                    if SMOBaseObject.warning:
                        SMOBaseObject.debug_handler.out(
                            '*** WARNING *** RCInit distribute_port_data: cant find port %s of out_succ node %s skip transfer' % (y, x))
                    continue
                key = mirrow_edge[y]  # source port name
                if key not in source_port_tab:
                    # no renamed target where to drop the data, skip it
                    if SMOBaseObject.warning:
                        SMOBaseObject.debug_handler.out(
                            '*** WARNING *** RCInit  distribute_port_data: cant find succ node %s  port %s' % (x, y))  # info
                    continue

                if target_init_tab[y][1] == smo_list and not source_init_tab[key][1] == smo_list and\
                   source_port_tab[key] is not None:
                    # a list in source port should not appended to the target port
                    outpara[y] = [source_port_tab[key]]
                else:
                    # port type handeled by set_port_data
                    outpara[y] = source_port_tab[key]
            target_node.set_port_data(target_init_tab, target_tab, data=outpara)
            for y in outpara:
                outpara[y] = None
        return

    def collect_port_data(self, target_port_tab=None, target_init_port_tab=None,
                          node_tab=None, connection_tab=None):
        # Get input data from output ports of connected nodes

        if node_tab is None or target_port_tab is None or target_init_port_tab is None or connection_tab is None:
            if SMOBaseObject.warning:
                SMOBaseObject.debug_handler.out(
                    '*** WARNING *** RCInit  collect_port_data: source for distribution not defined (act_inport,outport)')
            return

        for x in connection_tab:
            # x is node name in rename tab
            if x not in node_tab:
                # node does not exist in upper graph
                if SMOBaseObject.info:
                    SMOBaseObject.debug_handler.out(
                        '*** Warning *** collect_port_data cant find pred node %s' % x)
                continue
            source_node = node_tab[x]
            source_tab = node_tab[x].external_outport
            source_init_tab = node_tab[x].init_outport

            edge = connection_tab[x][0]  # rename
            inpara = connection_tab[x][2]

            source_node.get_port_data(source_init_tab, source_tab, data=inpara)
            source_init_tab = source_node.init_inport

            for y in inpara:
                if not (y in edge):
                    # no rename entry for port y in node x, skip transfer
                    if SMOBaseObject.warning:
                        SMOBaseObject.debug_handler.out(
                            '*** WARNING *** PKBaseGraph  collect_port_data: cant find port %s for pred node %s' % (y, x))
                    continue
                key = edge[y]
                if key not in target_port_tab:
                    # no renamed target where to drop the data, skip it
                    if SMOBaseObject.warning:
                        SMOBaseObject.debug_handler.out(
                            '*** WARNING *** PKBaseGraph  collect_port_data: cant find port %s for target node %s' % (y, x))
                    continue

                if target_init_port_tab[key][1] == smo_list:
                    if source_init_tab[y][1] == smo_list:
                        target_port_tab[key].extend(inpara[y])
                    else:
                        # source port is value or token
                        if inpara[y] is not None:
                            target_port_tab[key].append(inpara[y])
                else:
                    # target port is value or token
                    #target_port_tab[key] = inpara[y]
                    if inpara[y] is not None:
                        target_port_tab[key] = inpara[y]
                        trigger = True
                # reset auxiliary buffer for the next time
                inpara[y] = None
        return
