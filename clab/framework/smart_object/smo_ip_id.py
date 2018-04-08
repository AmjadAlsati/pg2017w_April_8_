################
#
# Modul:        smart_object
# File:         smo_ip_id.py
#
# Author:       Bernd Kleinjohann
# Created:      May 2014
# Version:      0.1
#
# Contents:     This module supports  the identification of remote objects.
#               classSMOIPID smo object representative on client side
#
#               get_unique_id_str produce an unique string as name for smo remote objects
#
#               IP_Call:            Send a call message to refered IPID object and waits for results (remote Call)
#               IP_Send:            Send a call message to refered IPID object(send and forget)
#               IP_Broadcast:       Send a call message to all IPID objects     (send and forget)
#               IP_Broadcast_All:   Send a call message to all IPID objects published at the same Server like the refered Objcet (send and forget)
#               IP_All:             Send a call message to all known IPID objects(client) and IPID(server) side (send and forget)
#
import os
import sys
from traceback import extract_tb
from threading import Condition
import socket
import datetime
from Queue import Queue
from threading import currentThread

from . import smo_util
from .smo_base_object import SMOBaseObject
from .smo_sync_object import SMOSyncObject


if __name__ == '__main__':
    smo_act_host = 'localhost'
else:
    smo_act_host = socket.gethostname()

# smo_act_host may contain more than one adress
# else: smo_act_host =  socket.getfqdn()

# special for upb net .win.cs. ask netadmin
# else: smo_host = smo_host[0: smo_host.find('win.')] + smo_host[ smo_host.find('win.')+4:]


def get_unique_id_str(smo_obj):
    """
    This function produces a unique string for the identification of remote objects.
    So a SMOIPID can be stored and identify an unique object
    """
    act_time = datetime.datetime.now()
    unique_id_str = 'SMO_' + str(SMOIPID.smo_PID) + '_' + str(smo_obj.get_obj_no())
    unique_id_str += act_time.strftime('_DATE_%Y_%j_%H_%M_%S_') + str(act_time.microsecond)
    return unique_id_str


# direct calls for call share h
###

class SMOIPID(SMOBaseObject):
    """
    This class realize an representative of client side for a smo object
    """

    smo_host = smo_act_host
    """Local process data Hostname, initialisation only once (not after restart SMO)"""
    smo_IP = socket.gethostbyname(smo_host)
    """Local process data Host IP, initialisation only once (not after restart SMO)"""
    smo_PID = os.getpid()
    """Local process data Process ID, initialisation only once (not after restart SMO)"""

    # server data
    if SMOBaseObject.info:
        smo_util.smo_debug_handler.out('*** INFO *** SMOIPID: serveradr: %s , %s, PID: %s' %
                                       (str(smo_host), str(smo_IP), str(smo_PID)))

    smo_client_server_lock = Condition(lock=None)
    """ Thread syncronisation for modifying client or server dict"""
    smo_client_dict = {}
    """List of SMO IP connections"""
    smo_server_dict = {}
    """List of SMO IP server"""

    def __init__(self, server_host=None, server_port=None, connection_name=None,
                 obj_id_str=None, glob=None, smo_init_mode='new', **kwd):

        SMOBaseObject.__init__(self, smo_init_mode=smo_init_mode, glob=glob, **kwd)
        if smo_init_mode == 'new':
            self.aux_call_attr = None
            """ auxiliary var for remote call"""

            if obj_id_str is None:
                self.obj_id_str = smo_util.smo_default_id_str
            else:
                self.obj_id_str = str(obj_id_str)

            if self.server_host is None:
                self.server_host = 'localhost'
            else:
                self.server_host = str(server_host)

            self.server_port = int(server_port)
            if self.server_port is None or self.server_port <= smo_util.smo_IP_min_port or self.server_port > smo_util.smo_IP_max_port:
                self.server_port = smo_util.smo_IP_default_server_port

            self.connection_name = None
            if connection_name is not None:
                self.connection_name = str(connection_name)

        return

    def __str__(self):
        # This string will be extended by subclasses
        retstr = SMOBaseObject.__str__(self)
        retstr += str(self.obj_id_str) + ' adr: ' + str((self.server_host, self.server_port))
        retstr += ' by connection_name: ' + str(self.connection_name)
        return retstr

    def get_restore_dict(self):
        """
        This method should return a dictionary which contain all persistent data
        for all members of restore eval(repr(restore)) has to work
        """
        ret_dict = SMOBaseObject.get_restore_dict(self)
        local_dict = {
            'server_host': self.server_host,
            'server_port': self.server_port,
            'connection_name': self.connection_name,
            'obj_id_str': self.obj_id_str}
        ret_dict.update(local_dict)
        if self.trace:
            mstr = '*** TRACE *** SMOIPID get_restore_dict: return %s' % str(ret_dict)
            self.debug_handler.out(mstr)
        return ret_dict

    def get_adress(self):
        return (self.server_host, self.server_port)

    def set_adress(self, server_host=None, server_port=None):
        self.server_host = server_host
        if self.server_host is None:
            self.server_host = 'localhost'
        self.server_port = server_port
        if self.server_port or self.server_port <= smo_util.smo_IP_min_port or self.server_port > smo_util.smo_IP_max_port:
            self.server_port = smo_util.smo_IP_default_server_port
        return

    def get_connection_name(self):
        return self.connection_name

    def set_connection_name(self, connection_name=None):
        self.connection_name = connection_name
        return

    def get_id_str(self):
        return self.obj_id_str

    def set_id_str(self, obj_id_str=None):
        if obj_id_str is not None:
            self.obj_id_str = str(obj_id_str)
        else:
            obj_id_str = smo_util.smo_default_id_str
        return

    def get_object(self):
        obj = None
        client = None
        server = None

        SMOIPID.smo_client_server_lock.acquire(blocking=1)
        if (self.server_host, self.server_port) in SMOIPID.smo_client_dict:
            client = SMOIPID.smo_client_dict[(self.server_host, self.server_port)]
        if(self.server_host, self.server_port) in SMOIPID.smo_server_dict:
            server = SMOIPID.smo_server_dict[(self.server_host, self.server_port)]
        SMOIPID.smo_client_server_lock.release()

        if server is not None:
            server.smo_obj_id_lock.acquire(blocking=1)
            if self.id_str in server.smo_obj_id_dict:
                obj = server.smo_obj_id_dict[self.id_str]
            client_server.smo_obj_id_lock.release()

        ### object not in client
        if obj is not None and client is not None:
            client.smo_obj_id_lock.acquire(blocking=1)
            if self.id_str in client.smo_obj_id_dict:
                obj = client.smo_obj_id_dict[self.id_str]
            client_server.smo_obj_id_lock.release()

        return obj

    def expose(self, IP_obj):  # publish ???

        client = None
        server = None
        ret = False

        SMOIPID.smo_client_server_lock.acquire(blocking=1)
        if (self.server_host, self.server_port) in SMOIPID.smo_server_dict:
            server = SMOIPID.smo_server_dict[(self.server_host, self.server_port)]
        if (self.server_host, self.server_port) in SMOIPID.smo_client_dict:
            client = SMOIPID.smo_client_dict[(self.server_host, self.server_port)]
        SMOIPID.smo_client_server_lock.release()

        if server is not None and IP_obj.id_str != smo_util.smo_server_id_str:
            server.smo_obj_id_lock.acquire(blocking=1)
            server.smo_obj_id_str_dict[IP_obj.id_str] = IP_obj
            server.smo_obj_ref_dict[IP_obj] = IP_obj.id_str
            ret = True
            server.smo_obj_id_lock.release()

        if client is not None and IP_obj.id_str != smo_util.smo_server_id_str:
            client.smo_obj_id_lock.acquire(blocking=1)
            client.smo_obj_id_str_dict[IP_obj.id_str] = IP_obj
            client.smo_obj_ref_dict[IP_obj] = IP_obj.id_str
            ret = True
            client.smo_obj_id_lock.release()

        return ret

    def hide(self):  # unpublish ???

        client = None
        server = None
        obj = None

        SMOIPID.smo_client_server_lock.acquire(blocking=1)
        if (self.server_host, self.server_port) in SMOIPID.smo_server_dict:
            server = SMOIPID.smo_server_dict[(self.server_host, self.server_port)]
        if (self.server_host, self.server_port) in SMOIPID.smo_client_dict:
            client = SMOIPID.smo_client_dict[(self.server_host, self.server_port)]
        SMOIPID.smo_client_server_lock.release()

        if server is not None:
            server.smo_obj_id_lock.acquire(blocking=1)
            if self.id_str != smo_util.smo_server_id_str and self.id_str in server.smo_obj_id_dict:
                obj = server.smo_obj_id_dict[self.id_str]
                del server.smo_obj_id_dict[self.id_str]
                if obj in server.smo_obj_ref_dict:
                    del server.smo_obj_ref_dict[obj]
            server.smo_obj_id_lock.release()

        if client is not None:
            client.smo_obj_id_lock.acquire(blocking=1)
            if self.id_str != smo_util.smo_server_id_str and self.id_str in client.smo_obj_id_dict:
                obj = client.smo_obj_id_dict[self.id_str]
                del client.smo_obj_id_dict[self.id_str]
                if obj in client.smo_obj_ref_dict:
                    del client.smo_obj_ref_dict[obj]
            client.smo_obj_id_lock.release()
        return obj

    def __getattr__(self, attr_name):
        self.aux_call_attr = attr_name
        return self._smo_ip_call

    def _smo_ip_call(self, *para_args, **para_kwd):
        """
        This method checks the SMO ID and handles a SMO IP call even if it is local or global
        """
        # init local vars
        method_name = self.aux_call_attr
        self.aux_call_attr = None
        server = None
        obj = None

        # method call, send message (no return) or broadcast (no return)
        call_type = 'IP_Call'
        if 'smo_call_type' in para_kwd and para_kwd['smo_call_type'] in (
                'IP_Call', 'IP_Send', 'IP_Broadcast', 'IP_All', 'IP_Broadcast_All'):
            call_type = para_kwd['smo_sync']
            ### del para_kwd['smo_sync']

        # call remote method, get connection object
        IP_send_obj = None

        # check condition
        #####

        # get connection object IP_send_obj (client server handler)
        # look for client or server
        SMOIPID.smo_client_server_lock.acquire(blocking=1)
        if self.connection_name is not None and (self.server_host, self.server_port) in SMOIPID.smo_server_dict:
            client_server = SMOIPID.smo_server_dict[(self.server_host, self.server_port)]

        elif self.connection_name is None and (self.server_host, self.server_port) in SMOIPID.smo_client_dict:
            client_server = SMOIPID.smo_client_dict[(self.server_host, self.server_port)]
            # set connection parameter
            if 'smo_IP_connection' in para_kwd:
                # print 'XXXXXXXX HALLO 3'
                para_kwd['smo_IP_connection'] = self.connection_name
        else:
            client_server = None
        SMOIPID.smo_client_server_lock.release()

        # if server look for handler (connection name)
        # no two locks at the same time
        if self.connection_name is not None and client_server is not None:
            client_server.handler_lock.acquire(blocking=1)
            if self.connection_name in client_server.handler_lst:
                # server.handler.return thread   smo_req_queue
                IP_send_obj = client_server.handler_lst[self.connection_name].smo_return_thr
            else:
                # no connection
                IP_send_obj = None
            client_server.handler_lock.release()
        else:
            # client smo_req_queue
            IP_send_obj = client_server

        if IP_send_obj is None:
            if self.error:
                mstr = '*** INFO *** SMOIPID.smo_ip_call: No connection to host %s, port %s, connection: %s'\
                       % (str(self.server_host), str(self.server_port), str(self.connection_name))
                self.debug_handler.out(mstr)
            ###
            # raise exeption ???
            ###
            return None

        # call remote method, send message
        if self.trace:
            self.debug_handler.out('*** glogal smo_ip_call: ', method_name, ' at: ',
                                   str(self.server_host, self.server_port, self.connection_name))

        call_thread = currentThread()
        # assert that a queue for ack exists
        if not hasattr(call_thread, 'smo_req_queue'):
            call_thread.smo_req_queue = Queue()
            if self.trace:
                mstr = '*** WARNING *** SMOIPID _smo_ip_call: create req queue for thread %s' % (str(call_thread))
                self.debug_handler.out(mstr)
        if not hasattr(call_thread, 'smo_req_lst'):
            call_thread.smo_req_lst = []
            if self.trace:
                mstr = '*** WARNING *** SMOIPID _smo_ip_call: create req list for thread %s' % (str(call_thread))
                self.debug_handler.out(mstr)

        # built call method or send message
        if call_type == 'IP_Call':
            SMOBaseObject.msg_id_lock.acquire(blocking=1)
            SMOBaseObject.msg_id_count += 1
            msg_ident = SMOBaseObject.msg_id_count
            SMOBaseObject.msg_id_lock.release()

            res_queue = call_thread.smo_req_queue
            res_lst = call_thread.smo_req_lst
        else:
            msg_ident = -1
            res_queue = None
            res_lst = None

        act_msg = (call_type, msg_ident, res_queue, self, method_name, para_args, para_kwd)

        ###
        ### 'IP_Call', 'IP_Send', 'IP_Broadcast', 'IP_All', 'IP_Broadcast_All'
        # Spaeter genauer festlegen ????
        ###

        if call_type in ('IP_Call', 'IP_Send', 'IP_All'):
         # IP_send_obj.smo_sync_use.acquire(blocking=1)
            if IP_send_obj.smo_req_queue is not None:
                IP_send_obj.smo_req_queue.put(act_msg)
            else:
                ###
                # raise exeption ???
                ###
                return False
# IP_send_obj.smo_sync_use.release()

        # handle Broadcast
        elif self.connection_name is not None and call_type in ('IP_Broadcast', 'IP_Broadcast_All'):
            client_server.handler_lock.acquire(blocking=1)
            for x in client_server.handler_lst:
                x.smo_sync_use.acquire(blocking=1)
                if x.smo_req_queue is not None:
                    x.smo_req_queue.put(act_msg)
                x.smo_sync_use.release()
            client_server.handler_lock.release()
        else:
            ###
            # raise exeption ???
            ###
            return False

        # exit for send and forget call types
        if call_type != 'IP_Call':
            return True

        # continue service until the results arrive otherwise a deadlock may occure
        while True:
            act_msg_lst = [x for x in res_lst if (x[0] in ('Return', 'IP_Return') and x[1] == msg_ident)]

            if len(act_msg_lst) == 0:
                ret_msg = res_queue.get()
            else:
                ret_msg = act_msg_lst[0]
                del act_msg_lst[0]
                if len(act_msg_lst) != 0 and self.error:
                    mstr = '*** WARNING *** SMOIPID _smo_ip_call: found multiple msg_ids %s' % (str(act_msg_lst))
                    self.debug_handler.out(mstr)

            if ret_msg[0] in ('Send', 'Call'):
                sync_type, call_ident, res_queue, sync_obj, method, para_args, para_kwd = ret_msg

                try:
                    #if self.trace: self.debug_handler.out('*** TRACE *** handle msg ', str((method.func_name,para_args, para_kwd)))
                    if self.trace:
                        self.debug_handler.out(
                            '*** TRACE *** SMOIPID _smo_ip_call: handle Call msg ', method.func_name)

                    # check method call
                    if sync_obj is not None:
                        retval = method(sync_obj, *para_args, **para_kwd)
                    else:
                        retval = method(*para_args, **para_kwd)
                    send_msg = ('Return', call_ident, None, retval)

                except Exception as Except:
                    error_list = extract_tb(sys.exc_info()[2])
                    error = error_list[len(error_list) - 1]
                    if self.error:
                        self.debug_handler.out(
                            '*** FATAL *** SMOIPID _smo_ip_call: SMOSyncObject %d call %s raises an exception:\n   %s\n   File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                            % (self.obj.obj_no, method.func_name, sys.exc_info()[1], error[0], error[1], error[2], error[3]))
                    send_msg = ('Return', call_ident, Except, None)
                if ret_msg[0] in ('Call',):
                    res_queue.put(send_msg)

            elif ret_msg[0] in ('Return', 'IP_Return'):
                instr, ident, exception, retval = ret_msg
                if ident == msg_ident:
                    # for x in return_lst:
                    # call_thread.smo_req_queue.put(x)
                    if exception is None:
                        return retval
                    else:
                        ###
                        # raise exeption ???
                        ###
                        return exception
                else:
                    res_lst.append(ret_msg)

            else:
                # undefined Message type, should never happen
                if self.error:
                    mstr = '*** ERROR *** SMOSequencer thr_handle_msg: get an undefined message: %s' % (thr_msg[0])
                    self.debug_handler.out(mstr)

        return None

# default objects


class SMOIPInterfaceObj(SMOSyncObject):
    """
    The  task of this synchronized object is to provide access to all published smo_objects
    """

    def __init__(self, server):
        self.server = server
        SMOSyncObject.__init__(self, id_str=smo_util.smo_server_id_str)

        self.server.smo_obj_id_lock.acquire(blocking=1)
        self.server.smo_obj_id_str_dict[smo_util.smo_server_id_str] = self
        self.server.smo_obj_ref_dict[self] = smo_util.smo_server_id_str
        self.server.smo_obj_id_lock.release()
        return

    def get_all_object(self):
        ret = []
        self.server.smo_obj_id_lock.acquire(blocking=1)
        for x in self.server.smo_obj_id_str_dict:
            obj_id = SMOIPID(id_str=x, server_host=self.server.host,
                             server_port=self.server.port, glob=self.server.glob)
            ret.append(obj_id)
        self.smo_obj_id_lock.release()
        return ret

    def print_call_para(self, arg_val, kwd_val):
        print '+++ call arg: ', str(arg_val), ' kwd: ', str(kwd_val)
        return (('+++ call return: ', arg_val, kwd_val))

    def print_send_para(self, arg_val, kwd_val):
        print '+++ send arg: ', str(arg_val), ' kwd: ', str(kwd_val)
        return (('+++ send return: ', arg_val, kwd_val))

# start main for testsequence
if __name__ == '__main__':
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ Start testsequence for SMOIPID')
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ For testsequence of SMOIPID start smo_ip_object.py')
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
