################
#
# Modul:        smart_object
# File:         smo_ip_out_handler.py
#
# Author:       Bernd Kleinjohann
# Created:      September 2014
# Version:      0.1
#
# Contents:     This File contains the classes for the communication via IP sockets
#
import sys
from builtins import str, object
from traceback import extract_tb

from . import smo_util


class SMOIPSendHandler(object):
    """ This class read messages from thread queues and send them to IP sockets
        expected members in client_server
            client_server is SMOBaseObject (debug)
            client_server.IP_call_lock
            client_server.IP_call_dict
            client_server.ip_address
        """

    def __init__(self, client_server, com, sock):

        self.com = com
        self.cl_ser = client_server
        self.sock = sock

        self.end = False
        return

    def IP_send(self):
        while not self.end:
            # get message from thread queue

            if self.cl_ser.smo_req_queue is not None:
                act_msg = self.cl_ser.smo_req_queue.get()
            else:
                act_msg = ('End', None)

            send_str = None
            send_data = None
            send_format = self.com.send_format

            # test output
            if self.com.verbous:
                self.com.debug_handler.out(
                    '*** VERBOUS *** SMOIPsendHandler address %s process message:\n   %s'
                    % ((str((self.com.server_host, self.com.server_port)), str(act_msg))))

            # decode message and prepare send data
            if act_msg[0] in ('IP_Call', 'IP_Send', 'IP_Broadcast', 'IP_All', 'IP_Broadcast_All'):
                msg_type, msg_ident, res_queue, IP_id, method_name, para_args, para_kwd = act_msg

                if 'smo_send_format' in para_kwd:
                    send_format = para_kwd['send_format']

                if msg_type == 'IP_Call':
                    self.com.IP_call_lock.acquire(blocking=1)
                    self.com.IP_call_dict[msg_ident] = (res_queue, send_format)
                    self.com.IP_call_lock.release()
                else:
                    # no acknoledge requiered
                    msg_ident = -1
                send_data = (msg_type, msg_ident, IP_id, method_name, para_args, para_kwd)

            elif act_msg[0] in ('Return', 'IP_Return'):
                msg_type, msg_ident, exception, retval = act_msg

                self.com.IP_return_lock.acquire(blocking=1)
                if msg_ident in self.com.IP_return_dict:
                    external_msg_ident = self.com.IP_return_dict[msg_ident]
                    send_data = ('IP_Return', external_msg_ident, exception, retval)
                    del self.com.IP_return_dict[msg_ident]
                else:
                    external_msg_ident = None
                self.com.IP_return_lock.release()

                if external_msg_ident is not None:
                    send_data = ('IP_Return', external_msg_ident, exception, retval)
                else:
                    send_data = None
                    send_str = None
                    err_str = '*** WARNING *** SMOIPsendHandler address: %s  get an illegal IP_Return message'\
                        % (str((self.com.server_host, self.com.server_port)))
                    if self.com.warning:
                        self.com.debug_handler.out(err_str)

            elif act_msg[0] == 'End':
                err_str = '*** INFO *** SMOIPsendHandler address %s end' % (
                    str((self.com.server_host, self.com.server_port)))
                if self.com.info:
                    self.com.debug_handler.out(err_str)

                # send return for open calls
                self.send_exceptions(err_str)

                self.end = True
                send_data = None
                send_str = None

            else:
                send_data = None
                send_str = None
                if self.com.warning:
                    self.com.debug_handler.out(
                        '*** WARNING *** SMOIPsendHandler for %s get illegal message  %s'
                        % (str((self.com.server_host, self.com.server_port)), str(send_data)))

            # test output
            if self.com.verbous:
                self.com.debug_handler.out(
                    '*** VERBOUS *** SMOIPsendHandler IP_send_call address %s send data string:\n   %s'
                    % ((str((self.com.server_host, self.com.server_port)), str(send_data))))

            # prepare send string
            if send_data is not None:
                if send_format == 'py_source':
                    try:
                        send_data = ('py_source', send_data)
                        send_str = repr(send_data) + '\n' + smo_util.smo_IP_separator
                    except:
                        error_list = extract_tb(sys.exc_info()[2])
                        error = error_list[len(error_list) - 1]
                        err_str = '*** WARNING *** SMOIPsendHandler address %s get no string representation exception\n Send_data: %s\n %s\n File: %s , Line number: %s ,\n   Method: %s , Statement: %s'\
                                  % (str((self.com.server_host, self.com.server_port)), str(send_data), sys.exc_info()[1], error[0], error[1], error[2], error[3])
                        if self.com.warning:
                            self.com.debug_handler.out(err_str)
                        send_data = None
                        send_str = None

                        if send_data[0] == 'IP_Call':
                            exception_msg = ('IP_Return', msg_ident, Exception(err_str), None)
                            res_queue.put(exception_msg)
                            self.com.IP_call_lock.acquire(blocking=1)
                            del self.com.IP_call_dict[msg_ident]
                            self.com.IP_call_lock.release()
                else:
                    err_str = '*** ERROR *** SMOIPsendHandler illegal send format address %s send data string:\n   %s'\
                              % (str((self.com.server_host, self.com.server_port)), send_str)
                    if self.com.error:
                        self.com.debug_handler.out(err_str)

                    if send_data[0] == 'IP_Call':
                        exception_msg = ('IP_Return', msg_ident, Exception(err_str), None)
                        res_queue.put(exception_msg)
                        self.com.IP_call_lock.acquire(blocking=1)
                        del self.com.IP_call_dict[msg_ident]
                        self.com.IP_call_lock.release()
                    send_data = None
                    send_str = None

            # test output
            if self.com.verbous:
                self.com.debug_handler.out(
                    '*** VERBOUS *** SMOIPsendHandler IP_send_call address %s send data string:\n   %s'
                    % ((str((self.com.server_host, self.com.server_port)), send_str)))

            # Send the data
            if send_str is not None and send_str != '':
                try:
                    # send input data to socket
                    self.sock.sendall(send_str)
                    send_str = None
                    send_data = None
                except:
                    error_list = extract_tb(sys.exc_info()[2])
                    error = error_list[len(error_list) - 1]
                    err_str = '*** TRACE *** SMOIPsendHandler address %s could not send data, exception\n  %s\n File: %s , Line number: %s ,\n   Method: %s , Statement: %s'\
                              % (str((self.com.server_host, self.com.server_port)), sys.exc_info()[1], error[0], error[1], error[2], error[3])
                    if self.com.trace:
                        self.com.debug_handler.out(err_str)

                    # terminate open calls
                    self.send_exceptions(err_str)

                    send_str = None
                    send_data = None
                    self.end = True
                    try:
                        # signals were set by the client handler in the next loop
                        # self.sock.shutdown(socket.SHUT_RDWR)
                        self.sock.close()
                        self.sock = None
                    except:
                        error_list = extract_tb(sys.exc_info()[2])
                        error = error_list[len(error_list) - 1]
                        if self.com.trace:
                            self.com.debug_handler.out(
                                '*** Trace *** SMOIPClient address %s send error close socket, exception\n  %s\n File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                                % (str((self.com.server_host, self.com.server_port)), sys.exc_info()[1], error[0], error[1], error[2], error[3]))
            # end while
        try:
            err_str = '*** ERROR *** SMOIPsendHandler address %s close connection %s ' % (
                str((self.cl_ser.server_host, self.cl_ser.server_port)))
            self.send_exceptions(err_str)
        except:
            pass

        self.cl_ser.ip_out_end.set()
        return  # run

    def send_exceptions(self, err_str):
        self.com.IP_call_lock.acquire(blocking=1)
        for key in list(self.com.IP_call_dict.keys()):
            (res_queue, send_format) = self.com.IP_call_dict[key]
            exception_msg = ('IP_Return', key, Exception(err_str), None)
            res_queue.put(exception_msg)
            del self.com.IP_call_dict[key]
            self.com.IP_call_lock.release()
        return


class SMOIPThrSendHandler(smo_util.threading.Thread, SMOIPSendHandler):
    """Threaded version of class SMOIPSendHandler"""

    def __init__(self, client_server, com, sock):

        smo_util.threading.Thread.__init__(self)
        self.setDaemon(1)

        SMOIPSendHandler.__init__(self, client_server, com, sock)

        self.start()
        return

    def run(self):
        if self.com.error:
            self.com.debug_handler.out(
                '*** TRACE *** SMOIPThrSendHandler Protocoll: %s on host: %s start Sendhandler loop'
                % (str(self.com.IP_prot), str((self.com.server_host, self.com.server_port))))

        self.IP_send()

        if self.com.error:
            self.com.debug_handler.out(
                '*** TRACE *** SMOIPThrSendHandler Protocoll: %s on host: %s stops Sendhandler loop'
                % (str(self.com.IP_prot), str((self.com.server_host, self.com.server_port))))
        return


# start main for testsequence
if __name__ == '__main__':
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ Start testsequence for SMOIPreceiveHandler')
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ For testsequence start smo_ip_object.py')
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
