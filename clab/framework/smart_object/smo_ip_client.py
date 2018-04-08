################
#
# Modul:        smart_object
# File:         smo_ip_client.py
#
# Author:       Bernd Kleinjohann
# Created:      May 2014
# Version:      0.1
#
# Contents:     This File contains the classes for the client side of
#               IP based remote procedure calls for smo_sync_objects
#
#               class SMOIPClient send request by IP to a server (internal)
#               class SMOIPClientHandler processes the rpc return data (internal)
#               SMO_IP_connect to a server (factory function, reconnect on exception)
#               SMO_IP_disconnect delete a sserver
#               SMO_IP_disconnect_all delete all sserver
#               SMO_IP_get_all_connection_adress returns all connected client adresses
#

### global imports
import socket
import sys
import threading
from Queue import Queue
from time import sleep
from threading import Condition, Event
from traceback import extract_tb

from . import smo_util
from .smo_base_object import SMOBaseObject
from .smo_ip_id import SMOIPID, SMOIPInterfaceObj
from .smo_ip_out_handler import SMOIPThrSendHandler
from .smo_ip_in_handler import SMOIPThrReceiveHandler


class SMOIPClient(SMOBaseObject, threading.Thread):
    """
    This class SMOIPClient send request by IP to a server (internal)
    Create an instance by the factory function SMO_IP_connect
    """

    def __init__(self, smo_init_mode='new',
                 server_host=smo_util.smo_IP_default_server_host, server_port=smo_util.smo_IP_default_server_port,
                 reconnect_period=smo_util.smo_IP_default_reconnect_period,
                 smo_send_format='py_source', IP_prot='TCP', glob=None, **kwd):

        if self.trace:
            mstr = '*** TRACE *** SMOIPClient init: host %s port: %s' % (str(server_host), str(server_port))
            self.debug_handler.out(mstr)

        # init base classes
        SMOBaseObject.__init__(self, glob=glob, **kwd)
        threading.Thread.__init__(self)
        self.setDaemon(1)

        # set server data
        self.IP_prot = IP_prot
        """ Possible protocoll types are 'TCP' (implemented) or 'UDP' (not implemented)"""
        self.send_format = smo_send_format
        """actual only py_source is realized extentions pickle, marschal json ...  """
        if 'send_format' in kwd:
            self.send_format = kwd['send_format']
        self.glob = glob
        """ init global class definitions for eval (message en-/decoding format)"""

        self.server_host = server_host
        """ host of server adress, usr parameter"""
        self.server_host_name = None
        """ host name of server"""
        self.server_host_IP = None
        """ host IP of server """
        self.server_port = server_port
        """ port of server adress"""
        self.reconnect_period = reconnect_period
        """delay between to reconnet attempts"""

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

        # handler data
        self.end = False
        """Flag to stop Client"""
        self.sock = None
        self.smo_req_queue = None
        """ Queue for cal mesages from IPID objects and for return messages from the execution of published sync Objects"""
        self.smo_sync_use = Condition(lock=None)
        """Synchronisation for Queue access"""

        self.is_connected = threading.Event()
        self.is_connected.clear()

        # handling sub threads
        self.ip_in_handler = None
        self.ip_in_end = Event()
        self.ip_in_end.set()
        self.ip_out_handler = None
        self.ip_out_end = Event()
        self.ip_out_end.set()

        # start service
        self.start()
        return

    def run(self):
        """ Client Side
            This thread reads call and return messages from an input queue and send them
            as in py source format by an IP socket to the server"""

        if self.info:
            self.debug_handler.out('*** INFO *** SMOIPClient start connection to %s '
                                   % (str((self.server_host, self.server_port))))

        while not self.end:

            if (not self.is_connected.isSet()) and not self.end:
                # self.end condition for handling sleep !!!
                # establish connection
                try:
                    if self.fatal:
                        self.debug_handler.out('*** TEST *** SMOIPClient start try connect to %s '
                                               % (str((self.server_host, self.server_port))))
                    # get actual host name and IP adress
                    self.server_host_IP = socket.gethostbyname(self.server_host)
                    self.server_host_name = socket.getfqdn(self.server_host_IP)

                    if self.IP_prot == 'TCP':
                        # try to connect
                        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.sock.connect((self.server_host_IP, self.server_port))

# elif self.com_prot == 'UDP':
##                        self.sock = socket.socket()
##                        self.sock.connect((self.server_host_IP, self.server_host_port))

                    else:
                        if self.error:
                            self.debug_handler.out(
                                '*** ERROR *** SMOIPClient unknown Protocoll: %s on host: %s '
                                % (str(self.IP_prot), str((self.server_host, self.server_port))))

                    self.ip_in_end.clear()
                    self.ip_out_end.clear()
                    self.is_connected.set()

                    self.smo_req_queue = Queue()

                    if self.test:
                        self.debug_handler.out('*** TEST *** SMOIPClient connected to host %s' %
                                               (str((self.server_host, self.server_port))))

                    self.ip_in_handler = SMOIPThrReceiveHandler(self, self, self.sock)
                    self.ip_out_handler = SMOIPThrSendHandler(self, self, self.sock)

                    if self.test:
                        self.debug_handler.out('*** TEST *** SMOIPClient  host %s handler stopped' %
                                               (str((self.server_host, self.server_port))))

                    # wait for exit handler
                    self.ip_in_end.wait(timeout=None)
                    self.ip_out_end.wait(timeout=None)
                    self.is_connected.clear()

                except:
                    self.is_connected.clear()
                    error_list = extract_tb(sys.exc_info()[2])
                    error = error_list[len(error_list) - 1]
                    err_str = '*** TRACE *** SMOIPClient host %s,  throws an exception, get no connection to host\n   %s\n File: %s , Line number: %s ,\n   Method: %s , Statement: %s'\
                              % (str((self.server_host, self.server_port)), sys.exc_info()[1], error[0], error[1], error[2], error[3])
                    if self.trace:
                        self.debug_handler.out(err_str)
                    try:
                        # self.sock.shutdown(socket.SHUT_RDWR)
                        self.sock.close()
                        self.sock = None
                    except:
                        error_list = extract_tb(sys.exc_info()[2])
                        error = error_list[len(error_list) - 1]
                        if self.trace:
                            self.debug_handler.out(
                                '*** TRACE *** SMOIPClient host %s, socket closec throws an expected exception\n  %s\n File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                                % (str((self.server_host, self.server_port)), sys.exc_info()[1], error[0], error[1], error[2], error[3]))

                    self.sock = None
                    self.smo_req_queue = None
                    self.ip_in_handler = None
                    self.ip_out_handler = None
                    self.ip_in_end.set()
                    self.ip_out_end.set()

                    # wait befor retry connect
                    if self.trace:
                        self.debug_handler.out(
                            '*** TRACE *** SMOIPClient host %s, sleeps for %s '
                            % (str((self.server_host, self.server_port)), str(self.ip_client.reconnect_period)))
                    sleep(self.reconnect_period)

            else:
                if self.error:
                    self.debug_handler.out(
                        '*** ERROR *** SMOIPClient host %s, illegal state in main loop connected and try connect'
                        % (str((self.server_host, self.server_port))))
                sleep(self.ip_client.reconnect_period)
            # end while exit

        try:
            if self.info:
                self.debug_handler.out(
                    '*** INFO *** SMOIPClient host %s, try to disconnect'
                    % (str((self.server_host, self.server_port))))

            self.is_connected.clear()
            req_queue = self.smo_req_queue
            self.smo_req_queue = None
            if req_queue is not None:
                req_queue.put(('End',))

            #sock = self.sock
            #self.sock = None

            try:
                self.sock.close()
                self.sock = None
            except:
                pass

            self.ip_out_end.wait(timeout=None)
            self.ip_in_end.wait(timeout=None)
            self.ip_in_handler = None
            self.ip_out_handler = None
        except:
            self.is_connected.clear()
            error_list = extract_tb(sys.exc_info()[2])
            error = error_list[len(error_list) - 1]
            if self.trace:
                self.debug_handler.out(
                    '*** Trace *** SMOIPClientHandler run host %s close cconnection exception\n  %s\n File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                    % (str((self.server_host, self.server_port)), sys.exc_info()[1], error[0], error[1], error[2], error[3]))

            #self.sock = None
            self.smo_req_queue = None
            self.ip_in_handler = None
            self.ip_out_handler = None
            self.ip_in_end.set()
            self.ip_out_end.set()

        err_str = '*** TRACE *** SMOIPClient host %s, disconnect'\
                  % (str((self.server_host, self.server_port)))
        if self.trace:
            self.debug_handler.out(err_str)

        # remodve handler from handler list
        SMOIPID.smo_client_server_lock.acquire(blocking=1)
        if (self.server_host, self.server_port) in SMOIPID.smo_client_dict:
            del SMOIPID.smo_client_dict[(self.server_host, self.server_port)]
        SMOIPID.smo_client_server_lock.release()

        if self.info:
            self.debug_handler.out(
                '*** INFO *** SMOIPClient stops connection to  %s' % str((self.server_host, self.server_port)))
        return

    def stop(self):
        # self.smo_sync_use.acquire(blocking=1)

        self.end = True
        try:
            # self.ip_in_handler.sock.shutdown(socket.SHUT_RDWR)
            self.ip_in_handler.sock.close()
            self.sock = None
            sleep(1)
        except:
            error_list = extract_tb(sys.exc_info()[2])
            error = error_list[len(error_list) - 1]
            if self.trace:
                self.debug_handler.out(
                    '*** TRACE *** SMOIPClientHandler stop  host: %s close cconnection exception\n  %s\n File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                    % (str((self.server_host, self.server_port)), sys.exc_info()[1], error[0], error[1], error[2], error[3]))

        self.ip_in_end.wait(timeout=None)

        try:
            self.smo_req_queue.put(('End',))
        except:
            pass

        # wait for exit handler
        self.ip_out_end.wait(timeout=None)

        # self.smo_sync_use.release()
        return

    def publish(self, obj):
        self.smo_obj_id_lock.acquire(blocking=1)

        if obj.id_str in self.smo_obj_id_str_dict and obj not in self.smo_obj_ref_dict:
            if self.warning:
                mstr = '*** WARNING *** SMOIPClient publish: Inconsistent object dicts, delete id_str entry'
                self.debug_handler.out(mstr)
            del self.smo_obj_id_str_dict[obj.id_str]

        if obj.id_str not in self.smo_obj_id_str_dict and obj in self.smo_obj_ref_dict:
            if self.warning:
                mstr = '*** WARNING *** SMOIPClient publish: Inconsistent object dicts, delete ref entry'
                self.debug_handler.out(mstr)
            del self.smo_obj_ref_dict[obj]

        if obj.id_str in self.smo_obj_id_str_dict and  obj in self.smo_obj_ref_dict and\
           (obj.smo_obj_id_str_dict[obj.id_str] != obj or self.smo_obj_ref_dict[obj] != obj.id_str):
            if self.warning:
                mstr = '*** WARNING *** SMOIPClient publish: Inconsistent object dicts, delete inconsistent entries'
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


def SMO_IP_connect(server_host=smo_util.smo_IP_default_server_host, server_port=smo_util.smo_IP_default_server_port,
                   reconnect_period=smo_util.smo_IP_default_reconnect_period, glob=None, **kwd):

    if server_host is None:
        host = SMOIPID.smo_host
        if smo_util.smo_info:
            smo_util.smo_debug_handler.out(
                '*** info *** SMO_IP_connect change to host %s port %s\n' % (str(server_host), str(server_port)))
    if server_port is None or not(smo_util.smo_IP_min_port <= server_port <= smo_util.smo_IP_max_port):
        if smo_util.smo_error:
            smo_util.smo_debug_handler.out(
                '*** Error *** SMO_IP_connect port out of bounce host %s port %s\n' %
                (str(server_host), str(server_port)))
        return None
    if (server_host, server_port) in SMOIPID.smo_client_dict:
        if smo_util.smo_warning:
            smo_util.smo_debug_handler.out(
                '*** Warning *** SMO_IP_connect client exists host %s port %s\n' %
                (str(server_host), str(server_port)))
        return SMOIPID.smo_client_dic[(server_host, server_port)]

    x = SMOIPClient(server_host=server_host, server_port=server_port,
                    reconnect_period=reconnect_period, glob=glob, **kwd)
    SMOIPID.smo_client_server_lock.acquire(blocking=1)
    SMOIPID.smo_client_dict[(server_host, server_port)] = x
    SMOIPID.smo_client_server_lock.release()
    if smo_util.smo_info:
        smo_util.smo_debug_handler.out(
            '*** INFO *** SMOIPID_connect to host %s port %s\n' % (str(server_host), str(server_port)))
    return x


def SMO_IP_disconnect(server_host, server_port):
    SMOIPID.smo_client_server_lock.acquire(blocking=1)
    if not (server_host, server_port) in SMOIPID.smo_client_dict:
        if smo_util.smo_warning:
            smo_util.smo_debug_handler.out(
                '*** WARNING *** SMOIPID_disconnect to host %s port %s\n' % (str(server_host), str(server_port)))
        SMOIPID.smo_client_server_lock.release()
        return False
    x = SMOIPID.smo_client_dict[(server_host, server_port)]
    del SMOIPID.smo_client_dict[(server_host, server_port)]
    SMOIPID.smo_client_server_lock.release()
    x.stop()
    if smo_util.smo_info:
        smo_util.smo_debug_handler.out(
            '*** INFO *** SMOIPID_disconnect to host %s port %s\n' % (str(server_host), str(server_port)))
    return


def SMO_IP_disconnect_all():
    for a, b in SMOIPID.smo_client_dict.keys():
        SMO_IP_disconnect(a, b)
    return


def SMO_IP_get_connection(server_host, server_port):
    SMOIPID.smo_client_server_lock.acquire(blocking=1)
    ret = None
    if (server_host, server_port) in SMOIPID.smo_client_dict:
        ret = SMOIPID.smo_client_dict[(server_host, server_port)]
    SMOIPID.smo_client_server_lock.release()
    return ret


def SMO_IP_get_all_connection_adress():
    SMOIPID.smo_client_server_lock.acquire(blocking=1)
    ret = SMOIPID.smo_client_dict.keys()
    SMOIPID.smo_client_server_lock.release()
    return ret

# start main for testsequence
if __name__ == '__main__':
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ Testsequence of smo_ip_client')
    smo_util.smo_debug_handler.out('+++ For testsequence of smo_ip_client start smo_ip_server.py')
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

    s_host = smo_util.smo_IP_default_server_host
    s_port = smo_util.smo_IP_default_server_port

    smo_util.smo_debug_handler.out('+++ connect to: %s , %s' % (s_host, s_port))

    SMO_IP_connect(server_host=s_host, server_port=s_port)
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

    sleep_time = 5
    smo_util.smo_debug_handler.out('+++ sleep for %d seconds ' % sleep_time)
    sleep(sleep_time)
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

    smo_util.smo_debug_handler.out('+++ create smo_ipid')
    ipid = SMOIPID(obj_id_str='smo_test_1', server_host=s_host, server_port=s_port)
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

    print ipid
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

    smo_util.smo_debug_handler.out('+++ IP call smo_ipid.sync_call_prt 765 otto1')
    res = ipid.sync_call_prt(765, n_para='otto1')
    smo_util.smo_debug_handler.out('+++ result; %s' % str(res))
    smo_util.smo_debug_handler.out('+++')
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

##    smo_debug_handler.out( '+++ IP call smo_ipid.sync_call_prt 766 otto2')
##    res= ipid.sync_call_prt(766, n_para='otto2')
##    smo_debug_handler.out( '+++ result; %s'%str(res))
##    smo_debug_handler.out( '+++')
##    smo_debug_handler.out( '+++++++++++++++++++++++++++++++++++++++++++++++')
##
##    smo_debug_handler.out( '+++ IP call smo_ipid.sync_call_prt 767 otto3')
##    res= ipid.sync_call_prt(767, n_para='otto3')
##    smo_debug_handler.out( '+++ result; %s'%str(res))
##    smo_debug_handler.out( '+++')
##    smo_debug_handler.out( '+++++++++++++++++++++++++++++++++++++++++++++++')

    sleep_time = 5
    smo_util.smo_debug_handler.out('+++ sleep for %d seconds ' % sleep_time)
    sleep(sleep_time)
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

    smo_util.smo_debug_handler.out('+++ disconnect to: %s , %s' % (s_host, s_port))
    SMO_IP_disconnect(s_host, s_port)
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++++++++++++++')

    smo_util.smo_debug_handler.out('+++ disconnect all')
    # SMO_IP_disconnect_all()
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')

    # sleep(30)

    smo_util.smo_debug_handler.out('+++ End of test')
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    exit
