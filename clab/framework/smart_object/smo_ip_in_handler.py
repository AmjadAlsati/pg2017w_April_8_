################
#
# Modul:        smart_object
# File:         smo_ip_in_handler.py
#
# Author:       Bernd Kleinjohann
# Created:      September 2014
# Version:      0.1
#
# Contents:     This File contains the classes for the communication via IP sockets
#
import sys
from builtins import str
from builtins import object
from traceback import extract_tb

from . import smo_util
from .smo_base_object import SMOBaseObject
from .smo_sync_object import SMOSyncObject


class SMOIPReceiveHandler(object):
    """ This class read messages from thread queues and send them to IP sockets
        expected members in client_server:

            client_server is SMOBaseObject (debug)
            client_server.ip_address

            client_server.IP_call_lock
            client_server.IP_call_dict

            client_server.IP_return_lock
            client_server.IP_return_dict
        """

    def __init__(self, client_server, com, sock):
        self.com = com
        self.cl_ser = client_server
        self.sock = sock
        self.glob = com.glob
        self.send_format = com.send_format
        self.end = False
        return

    def IP_receive(self):
        # init values
        receive_str = ''
        act_receive_str = ''
        py_msg_str = None
        py_IP_msg = None

        while not self.end:
            # get message from socket
            try:
                act_receive_str = None
                act_receive_str = self.sock.recv(smo_util.smo_IP_msg_length)

                if act_receive_str is None or act_receive_str == '':
                    raise Exception('receive empty string, disconnected')
                # connection exeption
                receive_str += act_receive_str
            except:
                error_list = extract_tb(sys.exc_info()[2])
                error = error_list[len(error_list) - 1]

                err_str = '*** TRACE *** SMOIPreceiveHandler handle for adress %s, receive throws an exception\n  %s\n File: %s , Line number: %s ,\n   Method: %s , Statement: %s'\
                    % (str((self.com.server_host, self.com.server_port)), sys.exc_info()[1], error[0], error[1], error[2], error[3])
                if self.com.trace:
                    self.com.debug_handler.out(err_str)

                # return for open calls can not be send (sock close)
                self.com.IP_return_lock.acquire(blocking=1)
                for x in list(self.com.IP_return_dict.keys()):
                    del self.com.IP_return_dict[x]
                self.com.IP_return_lock.release()

                self.end = True
                receive_str = ''
                act_receive_str = ''

            if receive_str is not None and receive_str != '':
                receive_msg_lst = receive_str.split(smo_util.smo_IP_separator, 1)
                if len(receive_msg_lst) == 1:
                    # no complete message available
                    py_msg_str = None
                    receive_str = receive_msg_lst[0]
                else:
                    # message ready for processing
                    py_msg_str = receive_msg_lst[0]
                    receive_str = receive_msg_lst[1]

            # decode message into python message
            if py_msg_str is not None and py_msg_str != '':
                # test output
                if self.com.trace:
                    self.com.debug_handler.out(
                        '*** TRACE *** SMOIPreceiveHandler address %s process python string:\n***   %s'
                        % ((str((self.com.server_host, self.com.server_port)), str(py_msg_str))))
                try:
                    if self.glob is None:
                        py_IP_msg = eval(py_msg_str)
                    else:
                        py_IP_msg = eval(py_msg_str, self.glob)
                    py_msg_str = None
                except:
                    # message evaluation error
                    error_list = extract_tb(sys.exc_info()[2])
                    error = error_list[len(error_list) - 1]
                    err_str = '*** ERROR *** SMOIPreceiveHandler address %s, can not eval message string exception:\n   %s\n   File: %s , Line number: %s ,\n   Method: %s , Statement: %s'\
                              % (str((self.com.server_host, self.com.server_port)), sys.exc_info()[1], error[0], error[1], error[2], error[3])
                    if self.com.error:
                        self.com.debug_handler.out(err_str)
                    py_IP_msg = None
                    # unknown msg_id, so no return message possible

            if py_IP_msg is not None:
                if py_IP_msg[0] == 'py_source':
                    act_msg = py_IP_msg[1]
                    py_IP_msg = None

                    if act_msg[0] in ('IP_Call', 'IP_Send', 'IP_Broadcast'):
                        msg_type, external_msg_ident, IP_id, method_name, para_args, para_kwd = act_msg

                        if msg_type == 'IP_Call':
                            obj_msg_type = 'Call'
                            smo_ack = True
                            SMOBaseObject.msg_id_lock.acquire(blocking=1)
                            SMOBaseObject.msg_id_count += 1
                            msg_ident = SMOBaseObject.msg_id_count
                            SMOBaseObject.msg_id_lock.release()

                            self.com.IP_return_lock.acquire(blocking=1)
                            self.com.IP_return_dict[msg_ident] = external_msg_ident
                            self.com.IP_return_lock.release()
                            res_queue = self.cl_ser.smo_req_queue

                        else:
                            obj_msg_type = 'Send'
                            smo_ack = False
                            msg_ident = -1
                            res_queue = None

                        obj = None
                        self.com.smo_obj_id_lock.acquire(blocking=1)

                        if IP_id.obj_id_str in self.com.smo_obj_id_str_dict:
                            obj = self.com.smo_obj_id_str_dict[IP_id.obj_id_str]
                        self.com.smo_obj_id_lock.release()

                        method = None
                        if obj is not None and hasattr(obj, method_name):
                            method = getattr(obj, method_name)

                        if method is not None and callable(method):
                            queue_msg = (obj_msg_type, msg_ident, res_queue, None, method, para_args, para_kwd)

                            if isinstance(obj, SMOSyncObject) and obj.sequencer[
                                    0] is not None and obj.sequencer[0].thr_serialize.isSet():
                                obj.sequencer[0].smo_req_queue.put(queue_msg)

                            elif SMOSyncObject.sequencer[0] is not None and SMOSyncObject.sequencer[0].thr_serialize.isSet():
                                SMOSyncObject.sequencer[0].smo_req_queue.put(queue_msg)
                            else:
                                try:
                                    if self.com.trace:
                                        self.debug_handler.out(
                                            '*** TRACE *** handle msg(ref) object %s call %s' %
                                            (str(IP_id), method.__name__))
                                    retval = method(*para_args, **para_kwd)
                                    ret_msg = ('IP_Return', msg_ident, None, retval)
                                except Exception as Except:
                                    error_list = extract_tb(sys.exc_info()[2])
                                    error = error_list[len(error_list) - 1]
                                    if self.com.error:
                                        self.com.debug_handler.out(
                                            '*** ERROR *** thr_handle_msg: SMOSyncObject %s call %s raises an exception:\n   %s\n   File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                                            % (str(IP_id), method.__name__, sys.exc_info()[1], error[0], error[1], error[2], error[3]))
                                    ret_msg = ('IP_Return', msg_ident, Except, None)
                                if obj_msg_type in ('Call', 'IP_Call'):
                                    res_queue.put(ret_msg)
                        else:
                            err_str = '*** WARNING *** SMOIPreceiveHandler address: %s  get no callable for Send/Call%s'\
                                % (str((self.com.server_host, self.com.server_port)), str(act_msg))
                            if self.com.warning:
                                self.com.debug_handler.out(err_str)

                            # error if no callable found
                            if msg_type == 'IP_Call':
                                exception_msg = ('IP_Return', msg_ident, Exception(err_str), None)
                                res_queue.put(exception_msg)

                    elif act_msg[0] in ('IP_All', 'IP_Broadcast_All'):
                        msg_type, external_msg_ident, IP_id, method_name, para_args, para_kwd = act_msg

                        obj_msg_type = 'Send'
                        smo_ack = False
                        msg_ident = -1
                        res_queue = None

                        self.com.smo_obj_id_lock.acquire(blocking=1)
                        obj_lst = list(self.com.smo_obj_ref_dict.values())
                        self.com.smo_obj_id_lock.release()

                        for obj in obj_lst:
                            method = None
                            if hasattr(obj, method_name):
                                method = getattr(obj, method_name)
                            if method is not None and callable(method):
                                queue_msg = (obj_msg_type, msg_ident, res_queue, obj, method, para_args, para_kwd)

                                if isinstance(obj, SMOSyncObject) and obj.sequencer[0] is not None:
                                    obj.sequencer[0].smo_req_queue.put(queue_msg)
                                else:
                                    SMOSyncObject.sequencer[0].smo_req_queue.put(queue_msg)

                                if isinstance(obj, SMOSyncObject) and obj.sequencer[
                                        0] is not None and obj.sequencer[0].thr_serialize.isSet():
                                    obj.sequencer[0].smo_req_queue.put(queue_msg)
                                elif SMOSyncObject.sequencer[0] is not None and SMOSyncObject.sequencer[0].thr_serialize.isSet():
                                    SMOSyncObject.sequencer[0].smo_req_queue.put(queue_msg)
                                else:
                                    try:
                                        if self.info:
                                            self.debug_handler.out(
                                                '*** INFO *** handle msg(ref) object %s call %s' %
                                                (str(IP_id), method.__name__))
                                        retval = method(*para_args, **para_kwd)
                                        ret_msg = ('IP_Return', msg_ident, None, retval)
                                    except Exception as Except:
                                        error_list = extract_tb(sys.exc_info()[2])
                                        error = error_list[len(error_list) - 1]
                                        if self.com.error:
                                            self.com.debug_handler.out(
                                                '*** ERROR *** thr_handle_msg: SMOSyncObject %d call %s raises an exception:\n   %s\n   File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                                                % (str(IP_id), method.__name__, sys.exc_info()[1], error[0], error[1], error[2], error[3]))
                                        ret_msg = ('IP_Return', msg_ident, Except, None)
                                    if obj_msg_type in ('Call', 'IP_Call'):
                                        res_queue.put(ret_msg)
                            else:
                                err_str = '*** WARNING *** SMOIPreceiveHandler address: %s  get no callable for IP_All: %s'\
                                    % (str((self.com.server_host, self.com.server_port)), str(act_msg))
                                if self.com.warning:
                                    self.com.debug_handler.out(err_str)

                    elif act_msg[0] in ('Return', 'IP_Return'):
                        msg_type, msg_ident, exception, retval = act_msg

                        self.com.IP_call_lock.acquire(blocking=1)
                        if msg_ident in self.com.IP_call_dict:
                            (res_queue, send_format) = self.com.IP_call_dict[msg_ident]
                            del self.com.IP_call_dict[msg_ident]
                        else:
                            res_queue = None
                        self.com.IP_call_lock.release()

                        if res_queue is not None:
                            queue_msg = ('IP_Return', msg_ident, exception, retval)
        # ???  exception    queue_msg = ('IP_Return', external_msg_ident, exception, IP_retval)
                            res_queue.put(queue_msg)
                        else:
                            err_str = '*** WARNING *** SMOIPreceiveHandler address: %s  get an illegal IP_Return message (msg_id) %s'\
                                % (str((self.com.server_host, self.com.server_port)), str(act_msg))
                            if self.com.warning:
                                self.com.debug_handler.out(err_str)

                        queue_msg = None
                        py_msg_str = None
                        py_IP_msg = None

                    elif act_msg[0] == 'End':
                        err_str = '*** TRACE *** SMOIPreceiveHandler address %s end' % (
                            str((self.com.server_host, self.com.server_port)))
                        if self.com.trace:
                            self.com.debug_handler.out(err_str)
                        self.end = True

                    else:
                        err_str = '*** ERROR *** SMOIPreceiveHandler illegal message type address %s send data string:\n   %s'\
                                  % (str((self.com.server_host, self.com.server_port)), str(py_IP_msg))
                        if self.com.error:
                            self.com.debug_handler.out(err_str)
                        py_msg_str = None
                        py_IP_msg = None
                else:
                    err_str = '*** ERROR *** SMOIPreceiveHandler illegal send format address %s send data string:\n   %s'\
                              % (str((self.com.server_host, self.com.server_port)), str(py_IP_msg))
                    if self.com.error:
                        self.com.debug_handler.out(err_str)
                    py_msg_str = None
                    py_IP_msg = None
            # end while
        try:
            # self.sock.shutdown(socket.SHUT_RDWR)
            self.sock.close()
            self.sock = None
        except:
            error_list = extract_tb(sys.exc_info()[2])
            error = error_list[len(error_list) - 1]
            if self.com.trace:
                self.com.debug_handler.out(
                    '*** Trace *** SMOIPreceiveHandler address %s close socket throws an exception\n  %s\n File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                    % (str((self.com.server_host, self.com.server_port)), sys.exc_info()[1], error[0], error[1], error[2], error[3]))

        # no more calls !!!
        self.cl_ser.smo_sync_use.acquire(blocking=1)
        res_queue = self.cl_ser.smo_req_queue
        self.cl_ser.smo_req_queue = None

        try:
            res_queue.put(('End',))
        except:
            error_list = extract_tb(sys.exc_info()[2])
            error = error_list[len(error_list) - 1]
            if self.com.trace:
                self.com.debug_handler.out(
                    '*** Trace *** SMOIPreceiveHandler address %s send end message throws an exception\n  %s\n File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                    % (str((self.com.server_host, self.com.server_port)), sys.exc_info()[1], error[0], error[1], error[2], error[3]))

        # acknoledge end

        self.cl_ser.smo_sync_use.release()
        self.cl_ser.ip_in_end.set()
        return


class SMOIPThrReceiveHandler(smo_util.threading.Thread, SMOIPReceiveHandler):
    """Threaded version of class SMOIPReceiveHandler"""

    def __init__(self, client_server, com, sock):

        smo_util.threading.Thread.__init__(self)
        self.setDaemon(1)

        SMOIPReceiveHandler.__init__(self, client_server, com, sock)
        self.com = com

        self.start()

    def run(self):

        if self.com.error:
            self.com.debug_handler.out(
                '*** TRACE *** SMOIPThrReceiveHandler Protocoll: %s on host: %s start Receivehandler loop'
                % (str(self.com.IP_prot), str((self.com.server_host, self.com.server_port))))

        self.IP_receive()

        if self.com.error:
            self.com.debug_handler.out(
                '*** TRACE *** SMOIPThrReceiveHandler Protocoll: %s on host: %s stops Receivehandler loop'
                % (str(self.com.IP_prot), str((self.com.server_host, self.com.server_port))))
        return

# start main for testsequence
if __name__ == '__main__':
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ Start testsequence for SMOIPreceiveHandler')
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ For testsequence start smo_ip_object.py')
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
