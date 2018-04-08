################
#
# Modul:        smart_object
# File:         smo_ip_server.py
#
# Author:       Bernd Kleinjohann
# Created:      May 2014
# Version:      0.1
#
# Contents:
"""
This File contains the classes for the server side of
IP based remote procedure calls for smo_sync_objects

class SMOIPServer starts the server and try to reconnect after the connection is lost
class SMOTCPServerHandler reads incomming IP requests and route them to the corresponding thread (internal)
class SMOIPServerReturn send back the returns of IP remote call (internal)
class SMOIPServerInterface an instance of this class allows access to a list of published objects

SMO_IP_new_server creates a new serv er for a given adress
SMO_IP_delete_server deletes a server for a given adress
SMO_IP_delete_all_server deletes all serfvers

SMO_IP_get_server returns a server instance for a given adress
SMO_IP_get_all_server_adress returns all adresses for which a server exists

SMO_IP_publish allow IP remote calls for an object by a server (may be used for more than one serverper object
SMO_IP_unpublish hide the object for a server for IP remote calls
"""
from future import standard_library
standard_library.install_aliases()
from builtins import str
import sys
from traceback import extract_tb
from threading import Condition, Event
import socketserver
from time import sleep
from queue import Queue

from . import smo_util
from .smo_base_object import SMOBaseObject
from .smo_sync_object import SMOSyncObject
from .smo_ip_id import SMOIPID, SMOIPInterfaceObj
from .smo_ip_out_handler import SMOIPThrSendHandler
from .smo_ip_in_handler import SMOIPThrReceiveHandler


class SMOIPServer(smo_util.threading.Thread, SMOBaseObject):
    """
    The task of this thread is to start, stop and retry an IP connection
    the TCPIP Server instance
    an dretry connection if connection is lost
    com_prot: ICPIP, UDP, LOCAL???
    """

    def __init__(self, server_host=smo_util.smo_IP_default_server_host, server_port=smo_util.smo_IP_default_server_port,
                 smo_send_format='py_source', IP_prot='TCP', reconnect_period=smo_util.smo_IP_default_reconnect_period, glob=None, **kwd):

        # init base classes
        smo_util.threading.Thread.__init__(self)
        SMOBaseObject.__init__(self, glob=glob, **kwd)
        self.setDaemon(1)

        self.IP_prot = IP_prot
        """ Possible protocoll types are 'TCP' (implemented) or 'UDP' (not implemented)"""
        self.send_format = smo_send_format
        """actual only py_source is realized possible extentions pickle, marschal json ...  """
        self.glob = glob
        """ init global class definitions for eval (message en-/decoding format)"""

        self.server_host = server_host
        """ host of server adress"""
        self.server_port = server_port
        """ port of server adress"""
        self.reconnect_period = reconnect_period
        """delay between to reconnet attempts"""

        # Socket server data
        self.server = None
        """Socket server object"""
        self.end = False
        """Flag to stop Server"""
        self.handler_lock = Condition(lock=None)
        """Synchronisation of the handler list"""
        self.handler_lst = []
        """list of all active handlers"""

        # table for redirect messages
        self.IP_return_lock = Condition(lock=None)
        """syncronisation for modifying ack_id_count"""
        self.IP_return_dict = {}
        """dict for mapping object calls (msg) from IPID to IP ack messages (return values) to the IPID objects
           via IP connection {ack_id_count:res_queue, ...}"""

        self.IP_call_lock = Condition(lock=None)
        """ Thread syncronisation for modifying IP_call_id_count"""
        self.IP_call_dict = {}
        """dict for mapping IP call to IP Return messages
        creat entry: when get a call msg is cfreated by an IPID
        value; ()
        use entry: when IP Return/ Exeption msg arrives"""

        # Data for pulished objects
        self.smo_obj_id_lock = Condition(lock=None)
        """syncronisation for modifieing smo_obj_id_str_dict or smo_obj_ref_dict"""
        self.smo_obj_id_str_dict = {}
        """ dict for object name : object"""
        self.smo_obj_ref_dict = {}
        """ dict for object : object name"""

        self.interface_obj = SMOIPInterfaceObj(self)
        """Initial ip object which provide acess to published ip objects"""

        # start service
        self.start()
        return

    def run(self):
        if self.test:
            self.debug_handler.out(
                '*** TEST *** SMOIPServer Protocoll: %s on host %s starts'
                % (str(self.IP_prot), str((self.server_host, self.server_port))))
        while not self.end:
            if self.info:
                self.debug_handler.out(
                    '*** INFO *** SMOIPServer Protocoll: %s on host %s create a new server'
                    % (str(self.IP_prot), str((self.server_host, self.server_port))))
            try:
                if self.IP_prot == 'TCP':
                    self.server = socketserver.ThreadingTCPServer(
                        (self.server_host, self.server_port), SMOTCPServerHandler)

# elif self.com_prot == 'UDP':
###                    self.server = SocketServer.ThreadingUDPServer((self.server_host,self.server_port), SMOUDPServerHandler)
###                    self.server = SocketServer.UDPServer((self.server_host,self.server_port), SMOUDPServerHandler)
                else:
                    if self.error:
                        self.debug_handler.out(
                            '*** ERROR *** SMOIPServer Protocoll: %s on host: %s port: %s unknown'
                            % (str(self.IP_prot), str(self.server_host), str(self.server_port)))
                self.server.smo_ip_server = self
                self.server.serve_forever()
            except:
                error_list = extract_tb(sys.exc_info()[2])
                error = error_list[len(error_list) - 1]
                if self.test:
                    self.debug_handler.out(
                        '*** TEST *** SMOIPServer Protocoll: %s on host: %s port: %s termination exception:\n   %s\n   File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                        % (str(self.IP_prot), str(self.server_host), str(self.server_port), sys.exc_info()[1], error[0], error[1], error[2], error[3]))
                self.server = None
                sleep(self.reconnect_period)

        if self.info:
            self.debug_handler.out(
                '*** INFO *** SMOIPServer Protocoll: %s on host: %s stops server'
                % (str(self.IP_prot), str((self.server_host, self.server_port))))
        return

    def stop(self):
        if self.test:
            self.debug_handler.out(
                '*** TEST *** SMOIPServer Protocoll: %s host %s server %s call stop'
                % (str(self.IP_prot), str(self.server_host), str(self.server_port)))
        self.end = True
        for x in self.handler_lst:
            if self.test:
                self.debug_handler.out(
                    '*** TEST *** SMOIPServer Protocoll: %s host %s server %s call stop handler'
                    % (str(self.IP_prot), str(self.server_host), str(self.server_port)))

# Queue und sync benutzen
            x.smo_end = True
            try:
                x.smo_req_queue.put(('End',))
            except:
                pass

            try:
                #### if x.smo_return_thr != None: x.smo_return_thr.stop()
                # x.request.shutdown(socket.SHUT_RDWR)
                x.request.close()
                x.request = None
            except:
                error_list = extract_tb(sys.exc_info()[2])
                error = error_list[len(error_list) - 1]
                if self.test:
                    self.debug_handler.out(
                        '*** TEST *** SMOIPServer  stop termination of connection %s:\n   %s\n   File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                        % (str(x.client_address), sys.exc_info()[1], error[0], error[1], error[2], error[3]))
        try:
            ser = self.server
            #self.server = None
            # ser.shutdown()
            self.server.server_close()
        except:
            error_list = extract_tb(sys.exc_info()[2])
            error = error_list[len(error_list) - 1]
            if self.test:
                self.debug_handler.out(
                    '*** TEST *** SMOIPServer Protocoll: %s host %s server %s termination exception:\n   %s\n   File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                    % (str(self.IP_prot), str(self.server_host), str(self.server_port), sys.exc_info()[1], error[0], error[1], error[2], error[3]))

        if self.info:
            self.debug_handler.out(
                '*** INFO *** SMOIPServer Protocoll: %s host %s server %s stop server'
                % (str(self.IP_prot), str(self.server_host), str(self.server_port)))
        return

    def publish(self, obj):
        self.smo_obj_id_lock.acquire(blocking=1)

        if obj.id_str in self.smo_obj_id_str_dict and obj not in self.smo_obj_ref_dict:
            if self.warning:
                mstr = '*** WARNING *** SMOIPServer publish: Inconsistent object dicts, delete id_str entry'
                self.debug_handler.out(mstr)
            del self.smo_obj_id_str_dict[obj.id_str]

        if obj.id_str not in self.smo_obj_id_str_dict and obj in self.smo_obj_ref_dict:
            if self.warning:
                mstr = '*** WARNING *** SMOIPServer publish: Inconsistent object dicts, delete ref entry'
                self.debug_handler.out(mstr)
            del self.smo_obj_ref_dict[obj]

        if obj.id_str in self.smo_obj_id_str_dict and  obj in self.smo_obj_ref_dict and\
           (obj.smo_obj_id_str_dict[obj.id_str] != obj or self.smo_obj_ref_dict[obj] != obj.id_str):
            if self.warning:
                mstr = '*** WARNING *** SMOIPServer publish: Inconsistent object dicts, delete inconsistent entries'
                self.debug_handler.out(mstr)
            del self.smo_obj_ref_dict[obj]
            del self.smo_obj_id_str_dict[obj.id_str]

        self.smo_obj_id_str_dict[obj.id_str] = obj
        self.smo_obj_ref_dict[obj] = obj.id_str
        self.smo_obj_id_lock.release()
        return

    def unpublish(self, obj):
        self.smo_obj_id_lock.acquire(blocking=1)
        if obj in self.smo_obj_ref_dict:
            del self.smo_obj_ref_dict[obj]
        if obj.id_str in self.smo_obj_id_str_dict:
            del self.smo_obj_id_str_dict[obj.id_str]
        self.smo_obj_id_lock.release()
        return


class SMOTCPServerHandler(socketserver.BaseRequestHandler):
    """
    reads IP messages and executethe requested call
    """

    def handle(self):
        smo_ip_server = self.server.smo_ip_server
        self.smo_client_address = self.client_address

        # insert self into handler list
        smo_ip_server.handler_lock.acquire(blocking=1)
        smo_ip_server.handler_lst.append(self)
        smo_ip_server.handler_lock.release()

        self.ip_in_end = Event()
        self.ip_in_end.clear()
        self.ip_out_end = Event()
        self.ip_out_end.clear()

        self.smo_req_queue = Queue()
        """ Queue for cal mesages from IPID objects and for return messages from the execution of published sync Objects"""
        self.smo_sync_use = Condition(lock=None)
        """Synchronisation for Queue access"""

        self.ip_in_handler = SMOIPThrReceiveHandler(self, smo_ip_server, self.request)
        self.ip_out_handler = SMOIPThrSendHandler(self, smo_ip_server, self.request)

        if smo_ip_server.test:
            smo_ip_server.debug_handler.out(
                '*** TEST *** SMOIPCServer connection handler to host %s started' % (str(self.smo_client_address)))

        # wait for exit handler

        self.ip_in_end.wait(timeout=None)
        self.ip_out_end.wait(timeout=None)

        if smo_ip_server.test:
            smo_ip_server.debug_handler.out(
                '*** TEST *** SMOIPCServer connection handler to host %s stoped' % (str(self.smo_client_address)))

        self.ip_in_handler = None
        self.ip_out_handler = None
        self.smo_req_queue = None

        # client connection finished
        smo_ip_server.handler_lock.acquire(blocking=1)
        if self in smo_ip_server.handler_lst:
            smo_ip_server.handler_lst.remove(self)
        smo_ip_server.handler_lock.release()

        if smo_ip_server.test:
            smo_ip_server.debug_handler.out(
                '*** TEST *** SMOTCPServerHandler stop handle for adress %s' % (str(self.smo_client_address)))
        return


def SMO_IP_new_server(server_host=smo_util.smo_IP_default_server_host, server_port=smo_util.smo_IP_default_server_port, IP_prot='TCP',
                      reconnect_period=smo_util.smo_IP_default_reconnect_period, glob=None, **kwd):

    if server_host is None:
        server_host = SMOIPID.smo_host
        if smo_util.smo_info:
            smo_util.smo_debug_handler.out(
                '*** info *** SMOIPID_new_server change to host %s port %s\n' % (str(server_host), str(server_port)))
    if server_port is None or not(smo_util.smo_IP_min_port <= server_port <= smo_util.smo_IP_max_port):
        if smo_util.smo_warning:
            smo_util.smo_debug_handler.out(
                '*** Warning *** SMOIPID_new_server port out of bounce to host %s port %s\n' %
                (str(server_host), str(server_port)))
        return None
    if (server_host, server_port) in SMOIPID.smo_server_dict:
        if smo_util.smo_warning:
            smo_util.smo_debug_handler.out(
                '*** Warning *** SMOIPID_new_server server exists host %s port %s\n' %
                (str(server_host), str(server_port)))
        return SMOIPID.smo_server_dict[(server_host, server_port)]

    x = SMOIPServer(server_host=server_host, server_port=server_port, IP_prot=IP_prot,
                    reconnect_period=reconnect_period, glob=glob, **kwd)

    SMOIPID.smo_client_server_lock.acquire(blocking=1)
    SMOIPID.smo_server_dict[(server_host, server_port)] = x
    SMOIPID.smo_client_server_lock.release()
    if smo_util.smo_trace:
        smo_util.smo_debug_handler.out(
            '*** TRACE *** SMOIPID_new_server to host %s port %s\n' % (str(server_host), str(server_port)))
    return x


def SMO_IP_delete_server(server_host, server_port):
    SMOIPID.smo_client_server_lock.acquire(blocking=1)
    if not (server_host, server_port) in SMOIPID.smo_server_dict:
        if smo_util.smo_test:
            smo_util.smo_debug_handler.out(
                '*** TEST *** SMOIPID_delete_server found no server host %s port %s\n' %
                (str(server_host), str(server_port)))
        SMOIPID.smo_client_server_lock.release()
        return False

    x = SMOIPID.smo_server_dict[(server_host, server_port)]
    del SMOIPID.smo_server_dict[(server_host, server_port)]
    SMOIPID.smo_client_server_lock.release()
    x.stop()
    if smo_util.smo_info:
        smo_util.smo_debug_handler.out(
            '*** INFO *** SMOIPID_delete_server delete server %s port %s' % (str(server_host), str(server_port)))
    return True


def SMO_IP_delete_all_server():
    for a, b in list(SMOIPID.smo_server_dict.keys()):
        SMO_IP_delete_server(a, b)
    return


def SMO_IP_get_server(server_host, server_port):
    SMOIPID.smo_client_server_lock.acquire(blocking=1)
    ret = None
    if (server_host, server_port) in SMOIPID.smo_server_dict:
        ret = SMOIPID.smo_server_dict[(server_host, server_port)]
    SMOIPID.smo_client_server_lock.release()
    return ret


def SMO_IP_get_all_server_adress():
    SMOIPID.smo_client_server_lock.acquire(blocking=1)
    ret = list(SMOIPID.smo_server_dict.keys())
    SMOIPID.smo_client_server_lock.release()
    return ret


def SMO_IP_publish(server_host, server_port, IP_obj):
    serv = SMO_IP_get_server(server_host, server_port)
    return serv.publish(IP_obj)


def SMO_IP_unpublish(server_host, server_port, IP_obj):
    serv = SMO_IP_get_server(server_host, server_port)
    return serv.unpublish(IP_obj)

# start main for testsequence
if __name__ == '__main__':
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ Testsequence of smo_ip_server')
    smo_util.smo_debug_handler.out('+++ For further Testsequences start smo_ip_client.py')
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

    # define test classes
    class test_ip_class_1(SMOSyncObject):

        def __init__(self, smo_init_mode='new', parent_thr_obj=None,
                     threaded=smo_util.smo_default_threaded, auto_serialize=smo_util.smo_default_serialize,
                     auto_compute=smo_util.smo_default_autocompute, period=smo_util.smo_default_period, glob=None,
                     parent=None, **kwd):
            self.data_1 = 'test_init_error'

            SMOSyncObject.__init__(self, smo_init_mode=smo_init_mode, parent_thr_obj=parent_thr_obj,
                                   threaded=threaded, auto_serialize=auto_serialize,
                                   auto_compute=auto_compute, period=period, glob=glob, **kwd)

            if smo_init_mode == 'new':
                self.data_1 = 'test_data'
                self.data_cnt = 0
                self.parent = parent  # application specific parent
            return

        def __str__(self):
            return SMOSyncObject.__str__(self)

        def get_restore_dict(self):
            ret_dict = SMOBaseObject.get_restore_dict(self)
            local_dict = {'data_1': self.data_1}
            ret_dict.update(local_dict)
            return ret_dict

        def restore_ref(self, test_para='', test_parent=None, **kwd):
            SMOBaseObject.restore_ref(self)
            if test_parent is None:
                smo_util.smo_debug_handler.out('### restore ref ', test_para, str(test_parent))
            else:
                smo_util.smo_debug_handler.out('### restore ref ', test_para)
            return

        def compute_handler(self, time_tab, comp_data='no_para', **kwd):

            smo_util.smo_debug_handler.out('### test_sync_class_1.compute_handler cycle: ', time_tab['act_cycle'] - time_tab['last_cycle'],
                                           ' period: ', time_tab['act_cycle_time'] - time_tab['last_cycle_time'], ' data_cnt: ', self.data_cnt, ' para: ', comp_data, str(kwd))
            return

        def print_data(self, para, n_para=''):
            smo_util.smo_debug_handler.out('###')
            smo_util.smo_debug_handler.out('### test_sync_1: %s, %s, %s, %s' % (
                str(para), str(n_para), str(self.data_1), str(self.parent)))
            return

        @smo_util.smo_sync_method_decorator
        def sync_call_prt(self, para, n_para=''):
            smo_util.smo_debug_handler.out('###')
            smo_util.smo_debug_handler.out('### sync_call_prt: %s, %s, %s, %s' %
                                           (str(para), str(n_para), str(self.data_1), str(self.parent)))
            return (17, 'egon')

        @smo_util.smo_sync_method_decorator
        def sync_send_prt(self, para, n_para='', wait=False):
            smo_util.smo_debug_handler.out('###')
            smo_util.smo_debug_handler.out('### sync_send_prt: %s, %s, %s, %s' %
                                           (str(para), str(n_para), str(self.data_1), str(self.parent)))
            return (18, 'willi')

# @smo_send_method_decorator
        @smo_util.smo_sync_method_decorator
        def sync_recursive(self, call='undef', level=0, order=0, ref_lst=()):
            smo_util.smo_debug_handler.out('### pre rec object %d call %s, level %d, order %d' %
                                           (self.obj_no, call, level, order))
            if level > 0:
                i = 0
                for x in ref_lst:
                    smo_util.smo_debug_handler.out('### start rec call: ', call, ' level ', level, ' order: ', i)
                    res = x.sync_recursive(call=call, level=level - 1, order=i, ref_lst=ref_lst)
                    smo_util.smo_debug_handler.out('### stop rec call: ', call, ' level ', level, ' order: ', i)
                    i += 1
            smo_util.smo_debug_handler.out('### post rec call %s, level %d, order %d' % (call, level, order))
            return level

        def set_data(self, para):
            self.data_1 = para
            return

        def get_data(self): return self.data1

    smo_util.smo_debug_handler.out('+++ create new default server')
    # SMO_IP_new_server(server_port=-1)
    test_server = SMO_IP_new_server(glob=globals())
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

    smo_util.smo_debug_handler.out('+++ create t_obj_1 (threadded)')
    t_obj_1 = test_ip_class_1(threaded=True, id_str='smo_test_1')
    # defaukt t_obj_1.start_serialize()
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

    t_obj_1.sync_call_prt(999, n_para='localtest')
    smo_util.smo_debug_handler.out('+++ publish t_obj_1')
    test_server.publish(t_obj_1)
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

    smo_util.smo_debug_handler.out('+++ create smo_ipid')
###
    idid = SMOIPID(id_str='smo_test_1', server_host='localhost', server_port=smo_util.smo_IP_default_server_port)
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

    sleep_time = 30
    smo_util.smo_debug_handler.out('+++ Serve for %d seconds ' % sleep_time)
    sleep(sleep_time)
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

    smo_util.smo_debug_handler.out('+++ delete all server')
    SMO_IP_delete_all_server()
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')

    smo_util.smo_debug_handler.out('+++ End of test')
    sleep(5)
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    exit
