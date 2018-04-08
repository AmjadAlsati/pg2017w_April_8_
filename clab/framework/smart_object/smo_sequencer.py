from future import standard_library
standard_library.install_aliases()
from builtins import str
################
#
# Modul:        PaderKicker Architekture
# File:         smo_sequencer.py
#
# Author:       Bernd Kleinjohann
# Created:      July 2009
# Version:      1.0
# Contents:     This file contains a class which sepuentialize method call for a thread
#               Once a call is active, the thread is able to perform futher requests
#               without blocking (extended worker model)
#

### global imports
import sys
from traceback import extract_tb
from threading import Thread
from threading import Event, Condition
from threading import currentThread
from queue import Queue

from .smo_base_object import SMOBaseObject


class SMOSequencer(Thread, SMOBaseObject):
    """
    This class implements a worker model for the SMO system. Calls of methods or sending messages
    were sequentialized. On request a special method will be called periodically.

    Remember: A python thread can not restart after run finishes. Therefor the class has destinct between
              stop_serialize and exit
              After an exeption the Idle environment may die
    """

    def __init__(self, obj, start_run=True):
        """Initialisation of serializer, starts run but not serialize"""
        if self.trace:
            mstr = '*** TRACE *** SMOSequencer init: object bo. %s' % str(obj.obj_no)
            self.debug_handler.out(mstr)
        if obj is None:
            if self.fatal:
                mstr = '*** FATAL *** SMOSequencer init: illegal SMO object'
                self.debug_handler.out(mstr)
            return None
        else:
            self.obj = obj
            """object to which the thread belongs to (back ref)"""
        Thread.__init__(self)
        SMOBaseObject.__init__(self, smo_init_mode='new')

        self.setDaemon(True)
        """ Thread initialisation"""
        self.smo_req_queue = Queue()
        self.smo_req_lst = []
        """ Requestqueue for call messages and evaluation cycles
        A thread may send messages to it self for evaluatin start/stop service etc.
        All requests have to served bevor stopping, otherwise a different thread may block
        the result is send to the req_queue of the calling process in order to avoid deadlocks.
        While a call from a serialized SMO thread will be performed, the calling SMO thread has to
        continue its service since the serialized thread may reques service
        by the calling thread"""
        self.open_calls = 0
        """indicates how many Return messages are expected. used for shutting down the thread
        (or exchange)"""
        self.thr_sync = Event()
        self.thr_sync.set()
        """Event for synchronisation of start/stopSerialize
        Every operation Call send ... has to wait on thr_sync
        when start/stopSerialize is requested fist ooperation is clear thr_sync so that
        all further operations are delayed until Start/StopSerialize has produce
        a consistent state. Last Aktion of Start/stop inside the thread is thr_sync.set()
        clear is called outside the thread
        set is calles inside the thread
        all services has first to wait"""
        self.thr_serialize = Event()
        """Event that indicate that serialization works"""
        self.thr_exit = False
        """ Flag that stops the run mehod.
        Since Py thread.run can only called once
        this class offers the distinction to stop serialization and do a resart
        or addionally to exist the run method (parameter for StopSerialisation)"""
        self.thr_start = False
        """Flag that indicates request for periodic computation of the SMO node"""
        self.sync_comp = False
        """Flag for scheduling periodic computation and service requests
        sync_comp == True: _THR_Set_sync_comp is NOT in the req_queue
        SynComp == False: _thr_set_sync_comp is send to req_queue and not"""
        # Start thread
        if start_run:
            self.start()
        return

    def run(self):
        """Method for managing the Requeue and the Resqueue
        This method loops until thr_exit is set to True"""

        while not self.thr_exit:
            # wait if serialization is requested loop for start/Restart leave loop on exit
            self.thr_serialize.wait(timeout=None)
            if self.info:
                mstr = '*** INFO *** SMOSequencer object no %s main loop started' % str(self.obj.obj_no)
                self.debug_handler.out(mstr)
            # enable service requests
            self.thr_sync.set()
            # Initialisation of sync_comp, _thr_set_sync_comp currently not requested by self
            self.sync_comp = True

            while self.thr_serialize.isSet():
                # main serializer loop
                if self.thr_start and not self.smo_req_queue.empty():
                    # node computation is started and there exist some requests

                    # send a message to self, requests which are queued befor this message were
                    self.sync_comp = False
                    self.smo_req_queue.put(('Send', -1, None, None, self._thr_set_sync_comp, (), {}))

                while self.thr_start and not self.sync_comp:
                    # serialize the next requests until _thr_set_sync_comp is performed
                    retmsg = self.thr_handle_msg(self.smo_req_queue, self.smo_req_lst, -1)
                    if retmsg is not None or self.smo_req_lst != []:
                        # inconsisten serialisation, a request could not be served, may block ???
                        if self.error:
                            mstr = '*** ERROR *** SMOSequencer run: object no %s get return message %s, %d, skip it (Deadlock ???)' %\
                                (str(self.obj.obj_no), retmsg[0], retmsg[1])
                            self.debug_handler.out(mstr)
                if self.thr_serialize.isSet():
                    # serialisation is requested and in the loop above no stop request happens
                    if self.thr_start:
                        # node evaluation is requested
                        if self.obj is None:
                            # no object , could only appear after delete
                            if self.error:
                                mstr = '*** ERROR *** SMOSequencer run: undefined object (None) for computation (deleted?)'
                                self.debug_handler.out(mstr)
                        else:
                            if self.info:
                                mstr = '*** INFO *** SMOSequencer run: object no  %s compute node' % str(
                                    self.obj.obj_no)
                                self.debug_handler.out(mstr)
                            # call the compute method in obj
                            self.obj.execute_compute()
                    else:
                        # node computation is NOT started, perform service
                        retmsg = self.thr_handle_msg(self.smo_req_queue, self.smo_req_lst, -1)
                        if retmsg is not None and self.smo_req_lst != []:
                            # inconsisten serialisation, a request could not be served, may block ???
                            if self.error:
                                # mstr = '*** ERROR *** SMOSequencer run: obect no %s get Return I %s, %d, skip it'%\
                                #       (str(self.obj.obj_no),retmsg[0],retmsg[1])
                                mstr = '*** ERROR *** SMOSequencer run: get Return I %s, %d, skip it' % (retmsg[
                                                                                                         0], retmsg[1])
                                self.debug_handler.out(mstr)
            # leave main serializer loop
            if self.info:
                mstr = '*** INFO *** SMOSequencer object no %s main loop stopped' % str(self.obj.obj_no)
                self.debug_handler.out(mstr)
        return

    def _thr_set_sync_comp(self):
        """run method send this method to itsself to initialize the next evaluation step of the model
        service request up to _thr_set_sync_comp call will be served befor next evaluation step
        take place"""
        if self.trace:
            mstr = '*** TRACE *** SMOSequencer _thr_set_sync_comp: object no %s' % str(self.obj.obj_no)
            self.debug_handler.out(mstr)
        self.sync_comp = True
        return

    def thr_handle_msg(self, thr_queue, thr_lst, msg_ident):
        """This method do the message execution of the requested Services from req_queue

        There is a distinction between requests from outside of a SMO thread or not
        Requests from inside are called Worker calls etc.
        Worker calls has to send addionally the destination req_queue(smo_Thr) and do not use the standard
        res_queue. The Results contain their own destination and an identifiere, so that
        recursive Calls in the SMO system couldbe handled. If a higher recusion level produces
        their results first, the result were send to the next higher level by the call method (see thr_call)"""

        thr_msg_lst = [x for x in thr_lst if (x[0] in ('Return', 'IP_Return') and x[1] == msg_ident)]

        if len(thr_msg_lst) == 0:
            thr_msg = thr_queue.get()
        else:
            ret_msg = thr_msg_lst[0]
            del thr_msg_lst[0]
            if len(thr_msg_lst) != 0 and self.error:
                mstr = '*** WARNING *** SMOIPID _smo_ip_call: found multiple msg_ids %s' % (str(act_msg_lst))
                self.debug_handler.out(mstr)

        if thr_msg[0] in ('Send', 'IP_Send', 'Call', 'IP_Call'):
            # Call by an external thread
            if thr_msg[0] in ('Call', 'IP_Call'):
                send_req = True
            else:
                send_req = False

            call_type, call_ident, res_queue, sync_obj, method, para_args, para_kwd = thr_msg

            if self.trace:
                mstr = '*** TRACVe *** SMOSequencer thr_handle_msg: object no  %s executs Call: %s' % (
                    str(self.obj.obj_no), method.__name__)
                self.debug_handler.out(mstr)
            try:
                if sync_obj is not None:
                    if self.info:
                        self.debug_handler.out('*** INFO *** handle msg(ref) object no %s call %s' %
                                               (str(self.obj.obj_no), method.__name__))
                    # self.debug_handler.out('### mit obj: ', method, '\n', para_args, '\n', para_kwd)
###
                    ###retval = method(sync_obj, *para_args, **para_kwd)
                    retval = method(*para_args, **para_kwd)
                else:
                    if self.info:
                        self.debug_handler.out('*** INFO *** handle msg object no %s call %s' %
                                               (str(self.obj.obj_no), method.__name__))
###
                    # self.debug_handler.out('### ohne obj: ', method, '\n', para_args, '\n', para_kwd)
                    ###retval = method(sync_obj, *para_args, **para_kwd)
                    retval = method(*para_args, **para_kwd)
            except Exception as Except:
                error_list = extract_tb(sys.exc_info()[2])
                error = error_list[len(error_list) - 1]

                if self.fatal:
                    self.debug_handler.out(
                        '*** FATAL *** thr_handle_msg: SMOSyncObject %d call %s raises an exception:\n   %s\n   File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                        % (self.obj.obj_no, method.__name__, sys.exc_info()[1], error[0], error[1], error[2], error[3]))

                if send_req:
                    res_queue.put(('Return', call_ident, Except, None))
                return None

            if send_req:
                res_queue.put(('Return', call_ident, None, retval))

            return None

        elif thr_msg[0] in ('Return', 'IP_Return'):
            # Results of a worker call, this results were handled by thr_call. run should never get such a message
            if self.trace:
                mstr = '*** TRACE *** SMOSequencer thr_handle_msg: object no %s get return message' % str(
                    self.obj.obj_no)
                self.debug_handler.out(mstr)
            if thr_msg[1] == msg_ident:
                return thr_msg
            else:
                thr_lst.append(thr_msg)
                return None
        else:
            # undefined Message type, should never happen
            if self.error:
                mstr = '*** ERROR *** SMOSequencer thr_handle_msg: object no %s get an undefined message: %s' %\
                       (str(self.obj.obj_no), thr_msg[0])
                self.debug_handler.out(mstr)
            return None

    def thr_call(self, call_type, sync_obj, sync_method, para_args, para_kwd):
        """ This method serialize a call of method with parameter args and kwd. wait indicates, if
            the calling method wait for the results of the call, ack indicates if the return message
            is automatically produced by run or externally(IP Call)"""

        if self.trace:
            mstr = '*** TRACE *** SMOSequencer thr_call: object no %s' % str(self.obj.obj_no)
            self.debug_handler.out(mstr)

        # wait for a consistent Serializer state may only temorary disabled
        self.thr_sync.wait(timeout=None)
        # determin Call for Worker destinktion
        call_thread = currentThread()

        if (not self.thr_serialize.isSet()) or call_thread == self:
            # no serialisation requested or necessary, performe a direct call
            if self.info:
                mstr = '*** INFO *** SMOSequencer thr_call: object no %s call %s' % (
                    str(self.obj.obj_no), sync_method.__name__)
                self.debug_handler.out(mstr)
            try:
                retval = None
                if sync_obj is not None:
                    if self.info:
                        self.debug_handler.out('*** INFO *** handle msg(ref) object no %s call %s' %
                                               (str(self.obj.obj_no), sync_method.__name__))
                    retval = sync_method(sync_obj, *para_args, **para_kwd)
                else:
                    if self.info:
                        self.debug_handler.out('*** INFO *** handle msg object no %s call %s' %
                                               (str(self.obj.obj_no), sync_method.__name__))
                    retval = sync_method(*para_args, **para_kwd)

                if call_type in ('Call', 'IP_Call'):
                    return retval
                else:
                    return None
            except Exception as Except:
                error_list = extract_tb(sys.exc_info()[2])
                error = error_list[len(error_list) - 1]
                if self.error:
                    self.debug_handler.out(
                        '*** ERROR *** thr_handle_msg: SMOSyncObject %d call %s raises an exception:\n   %s\n   File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                        % (self.obj.obj_no, sync_method.__name__, sys.exc_info()[1], error[0], error[1], error[2], error[3]))

                if call_type in ('Call', 'IP_Call'):
                    return retval
                else:
                    return None
        else:
            # Call an external thread
            msg_ident = -1
            res_queue = None
            res_lst = None
            send_req = False

            if call_type in ('Call', 'IP_Call'):
                send_req = True

                SMOBaseObject.msg_id_lock.acquire(blocking=1)
                SMOBaseObject.msg_id_count += 1
                msg_ident = SMOBaseObject.msg_id_count
                SMOBaseObject.msg_id_lock.release()

                if not hasattr(call_thread, 'smo_req_queue'):
                    call_thread.smo_req_queue = Queue()
                    if self.info:
                        mstr = '*** INFO *** SMOSequencer thr_call: object no %s create req queue for thread %s' % (
                            str(self.obj.obj_no), str(call_thread))
                        self.debug_handler.out(mstr)
                if not hasattr(call_thread, 'smo_req_lst'):
                    call_thread.smo_req_lst = []
                    if self.info:
                        mstr = '*** INFO *** SMOSequencer thr_call: object no %s create req list for thread %s' % (
                            str(self.obj.obj_no), str(call_thread))
                        self.debug_handler.out(mstr)

                res_queue = call_thread.smo_req_queue
                res_lst = call_thread.smo_req_lst

                # increase rekusice level
                self.open_calls += 1

            #if self.trace: self.debug_handler.out('*** TRACE *** thr call ', str((method.func_name, args, kwd)))
            self.smo_req_queue.put((call_type, msg_ident, res_queue, sync_obj, sync_method, para_args, para_kwd))

            if self.trace:
                mstr = '*** TRACE *** SMOSequencer thr_call: object no %s serialize worker call: %s' %\
                       (str(self.obj.obj_no), sync_method.__name__)

            # if no return message is  required
            if not send_req:
                return True

            # wait for return mesage and continue service
            while True:
                retmsg = self.thr_handle_msg(res_queue, res_lst, msg_ident)

                if retmsg is not None:
                    # the expected result message arrived (ident == msg_ident)
                    mstr = '*** TRACE *** SMOSequencer thr_call: Object no %s serialize call proceed %s, %s' %\
                           (str(self.obj.obj_no), str(retmsg[0]), str(retmsg[1]))

                    instr, ident, exception, retval = retmsg
                    self.open_calls -= 1

                    if exception is None:
                        return retval
                    else:
                        if self.error:
                            mstr = '*** ERROR *** SMOSeriaizer thr_call: object no %s call %s raises an exception:\n    %s'\
                                   % (str(self.obj.obj_no), str(sync_method.__name__), str(exception))
                            self.debug_handler.out(mstr)
                        return exception
                else:
                    # no return for this call (other return or call
                    if self.trace:
                        mstr = '*** TRACE *** SMOSequencer thr_call: Object no %s message (return_lst)  %s' %\
                               (str(self.obj.obj_no), str(retmsg))
                        self.debug_handler.out(mstr)

        if self.error:
            self.debug_handler.out('*** ERROR *** SMOSequencer thr_call: Should never happen')
        return None

    def is_serialized(self):
        return self.thr_serialize.isSet()

    def start_serialize(self):
        """Starts or restart the synchronization for this node."""

        if not self.thr_serialize.isSet():
            ### Thr is not running
            self.thr_sync.clear()
            # setted when run is operational
            self.thr_serialize.set()
            # start the main loop in run
            if self.trace:
                mstr = '*** TRACE *** SMOThr start_serialize: object no %s serializer started' % str(self.obj.obj_no)
                self.debug_handler.out(mstr)
            return True
        else:
            ### thr is running
            if self.warning:
                mstr = '*** WARNING *** SMOThr start_serialize: Node %s serializer is already running' % str(
                    self.obj.obj_no)
                self.debug_handler.out(mstr)
            return False

    def stop_serialize(self, **kwd):
        """This method stops the synchronization between the threads without stopping the
        run method of this thread. So the synchronisation could be restarted"""

        if self.thr_serialize.isSet():
            ### thr is running
            self.thr_sync.clear()
            # delay other requests until serializer stopped (continue working without serialisation)

            self.smo_req_queue.put(('Send', -1, None, None, self._thr_stop_serialize, (), kwd))
            # send stop instruktion to the thread

            self.thr_sync.wait(timeout=None)
            # wait on stop condition
            if self.trace:
                mstr = '*** TRACE *** SMOSequencer stop_serialize object no %s' % str(self.obj.obj_no)
                self.debug_handler.out(mstr)
            return True
        else:
            ### thr is stopped
            if self.warning:
                mstr = '*** WARNING *** SMOSequencer stop_serialize: object no %s sequencer not running' % str(
                    self.obj.obj_no)
                self.debug_handler.out(mstr)
            return False

    def _thr_stop_serialize(self, thr_exit=False, thr_new_sequencer=None):
        """stop_serialize send this method to itsself to stop the synchronization """
        # stop new serialization service
        self.thr_serialize.clear()
        # finish all  existing service requests

        while not self.smo_req_queue.empty():
            # execute pending calls
            retmsg = self.thr_handle_msg(self.smo_req_queue, self.smo_req_lst, -1)
            # there exist some higher level active calls get a return for such a call
            if retmsg is not None:
                if self.error:
                    mstr = '*** ERROR *** SMOSequencer _thr_stop_serialize: object no %s get Return %s, %d, should not happen' %\
                           (str(self.obj.obj_no), retmsg[0], retmsg[1])
                    self.debug_handler.out(mstr)
                # store message (should be return message

        # enable not synchronized services
        self.thr_sync.set()
        # check if run shoud realy finish
        if thr_exit:
            # no restart possible
            self.thr_exit = True
            if self.trace:
                mstr = '*** TRACE *** SMOSequencer _thr_stop_serialize: object no %s sequencer EXIT (no restart)' % str(
                    self.obj.obj_no)
                self.debug_handler.out(mstr)
        else:
            # restart should be possible
            if self.trace:
                mstr = '*** TRACE *** SMOThr _thr_stop_serialize: object no %s serializer stopped' % str(
                    self.obj.obj_no)
                self.debug_handler.out(mstr)
        return
