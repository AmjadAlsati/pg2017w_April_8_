################
#
# Modul:    smart_object
# File:     smo_syc_object.py
#
# Author:   Bernd Kleinjohann
# Created:  February 2014
# Version:  0.1
#
# Contents: This File contains a base class for all SMOSyncObject. An object may
#           span an own thread. The method task may run periodically. method calls
#           were sequentialized by a worker model. Class inherits from SMOBaseObject
#
# To Do:    connection to ip objects
import sys
from traceback import extract_tb
from threading import Event
from atexit import register
from time import sleep

from . import smo_util
from .smo_base_object import SMOBaseObject, smo_set_debug_options
from .smo_sequencer import SMOSequencer


if __name__ == '__main__':
    smo_set_debug_options(trace=False, info=False)
    # smo_set_debug_options(info=False)
    # smo_set_debug_options(debug=False)

##from smo_sequencer import SMOSequencer


class SMOSyncObject(SMOBaseObject):
    """This File contains a base class for all threadded SMO Objects """

    # global class data
    thr_call_sync = Event()
    """Synchronisation for calls of different threads"""
    sequencer = [None]
    """default sequencer for all  SMOSyncObjects, set and start after class definition"""
    obj_no = -3
    """compatibility definition for default sequencer"""

    def __init__(self, smo_init_mode='new', smo_restore=None, id_str=smo_util.smo_default_id_str,
                 smo_filename=None, smo_file=None, parent_thr_obj=None,
                 threaded=smo_util.smo_default_threaded, auto_serialize=smo_util.smo_default_serialize,
                 auto_compute=smo_util.smo_default_autocompute, period=smo_util.smo_default_period, glob=None, **kwd):
        """
        Initalisation of SMOSyncObject, pass parameters: smo_restore=None, smo_filename=None,
        smo_file=None to SMOBaseObject. Periodic execution is only allowed when threadded =True
        Otherwise method calls were synchronized by the parent_thr_obj. This ref may be indirect.
        """

        self.id_str = id_str
        """Id name for IP access pubilsh and IP Id """
        # init default values
        self.threaded = threaded
        """Flag that indicates if the the object spans an own thread"""
        # same as self.sequencer[0].obj == self
        self.auto_serialize = auto_serialize
        """Flag that indicates if the the object automatically starts serialisation"""
        self.auto_compute = auto_compute
        """flag indicates that computation should automatically start after initialisation (persistent) """
        self.period = period
        """period for calling compute_handler"""

        # init base object
        SMOBaseObject.__init__(self, smo_init_mode=smo_init_mode, smo_restore=smo_restore,
                               smo_filename=smo_filename, smo_file=smo_file, glob=glob, **kwd)

        self.run_compute = False
        """runtime flag that indicates periodic execution of compute handler"""

        # init timing data
        self.cycle_cnt = 0
        """executed cycles since start"""
        self.delay_time = 0
        """accumulated stop time"""
        self.start_time = 0
        """system time since first start or start after reset"""
        self.stop_time = 0
        """system time of the last stop call"""
        self.last_call = 0
        """system time of the last cycle at the beginning"""
        self.last_cnt = 0
        """cycle count of the previous executed cycle"""
        self.act_call = 0
        """system time of the actual cycle at the beginning"""
        self.act_cnt = 0
        """actual cycle count"""

        # synchronize sequencer for possible changes
        self.thr_call_sync = Event()
        self.thr_call_sync.set()
        # determin execution thread
        if not self.threaded:
            if parent_thr_obj is None or parent_thr_obj == SMOSyncObject:
                self.sequencer = SMOSyncObject.sequencer
            elif isinstance(parent_thr_obj, SMOSyncObject):
                self.sequencer = parent_thr_obj.sequencer
            else:
                self.sequencer = SMOSyncObject.sequencer
                if self.warning:
                    mstr = '*** WARNING *** SMOSyncObject init: Type %s, obj_no %s: Could not find parent thread %s, set it to SMOSyncObject'\
                        % (self.__class__.__name__, str(self.obj_no), str(obj_index))
                    self.debug_handler.out(mstr)
        else:
            # list used for propagation of in sequencer in the hierarchy
            self.sequencer = [SMOSequencer(self)]

        # default start serialization
        if self.threaded and self.auto_serialize:
            self.sequencer[0].start_serialize()
            # default start periodic execution
            if self.auto_compute and self.period >= 0:
                self.start_compute()
        return

    def __str__(self):
        """Produces a string containing the type of the SMOSyncObject"""
        # This string will be extended by subclasses
        retstr = SMOBaseObject.__str__(self)
        retstr += ' IP Id String: ' + self.id_str
        if self.threaded:
            retstr += ' threaded, period: %s' % str(self.period)
        else:
            retstr += ' serialized by object no %s' % str(self.sequencer[0].obj.obj_no)
        return retstr

    def get_restore_dict(self):
        """
        This method should return a dictionary which contain all persistent data
        for all members of restore eval(repr(restore)) has to work
        """
        ret_dict = SMOBaseObject.get_restore_dict(self)
        local_dict = {
            'id_str': self.id_str,
            'threaded': self.threaded,
            'auto_serialize': self.auto_compute,
            'auto_compute': self.auto_compute,
            'period': self.period}
        ret_dict.update(local_dict)
        if self.trace:
            mstr = '*** TRACE *** SMOSyncObject get_restore_dict: return %s' % str(ret_dict)
            self.debug_handler.out(mstr)
        return ret_dict

    def get_time_table(self):
        time_tab = {
            'start_time': self.start_time,
            'stop_time': self.stop_time,
            'act_cycle': self.act_cnt,
            'act_cycle_time': self.act_call,
            'last_cycle': self.last_cnt,
            'last_cycle_time': self.last_call}
        return time_tab

    def get_id_str(self):
        return self.id_str

    def set_id_str(self, id_str):
        if id_str != smo_util.smo_server_id_str:
            self.id_str = id_str
        return self.id_str

    # handling of serialize and compute by own methods
    def config(self, **kwd):
        if kwd == {}:
            kwd['auto_serialize'] = self.auto_serialize,
            kwd['auto_compute'] = self.auto_compute,
            kwd['threaded'] = self.threaded
            kwd['period'] = self.period
            kwd['id_str'] = self.id_str
            kwd['cycle_cnt'] = self.cycle_cnt               # read only
            kwd['delay_time'] = self.delay_time             # read only
            kwd['time_tab'] = self.get_time_tab()           # read only
            kwd['sync_obj'] = self.sequencer[0].obj
            return kwd
        else:
            ret_flag = True
            if 'auto_serialize' in kwd:
                self.auto_serialize = bool(kwd['auto_serialize'])
            if 'auto_compute' in kwd:
                self.auto_compute = bool(kwd['auto_compute'])
            if 'period' in kwd:
                self.period = int(kwd['period'])
            if 'id_str' in kwd and kwd['id_str'] != smo_util.smo_server_id_str:
                self.id_str = str(kwd['id_str'])

            # check threaded option
            if 'threaded' in kwd:
                threaded = bool(kwd['threaded'])
            else:
                threaded = None

            # replace sequencer by a new own one
            if threaded and self.threaded == False:

                if self.info:
                    mstr = '*** INFO *** SMOSyncObject config for object no %s: start own sequencer' % str(self.obj_no)
                    self.debug_handler.out(mstr)

                if self.sequencer[0] is None:
                    self._set_threaded()
                else:
                    self.thr_call_sync.reset()
                    self.sequencer[0].thr_call('Call', None, self._set_threaded, (), {})

            # stop own sequencer and replace it
            # looking for the requested new sequencer in option sync_object
            # otherwise use default SMOSequencer

            if not threaded:

                # check sync_obj option
                if 'sync_obj' in kwd and isinstance(kwd['sync_obj'], SMOSyncObject):
                    sync_obj = kwd['sync_obj']
                else:
                    sync_obj = SMOSyncObject

                # check if sequencer is different to the existing one
                if sync_obj.sequencer[0] != self.sequencer[0]:
                    if self.info:
                        mstr = '*** INFO *** SMOSyncObject config for object no %s: change sequencer' % str(
                            self.obj_no)
                        self.debug_handler.out(mstr)

                    # install new sequencer
                    if self.sequencer[0] is None:
                        self.sequencer = sync_obj.sequencer
                    else:
                        # change sequencer to an different sync_object OR (see _reset_threaded)
                        # stop own Thread (self.threaded == True)
                        self.thr_call_sync.reset()

                        self.sequencer[0].thr_call('Call', None, self._reset_threaded, (), {parent: sync_obj})
        return

    # auxiliary methods to start/stop sequencers
    def _reset_threaded(self, parent=None, thr_exit=True, **kwd):
        # no pending calls (thr_call_sync.reset)

        # stop own thread
        if self.sequencer[0] is not None and self.threaded and self.sequencer[0] != SMOSyncObject.sequencer[0]:
            self.sequencer[0]._thr_stop_serialize(thr_exit=thr_exit, thr_new_sequencer=parent.sequencer[0])

        # update own data
        self.sequencer = parent.sequencer
        self.threaded = False

        # enable remote calls
        self.thr_call_sync.set()
        return

    def _set_threaded(self, parent=None, **kwd):
        # no pending calls (thr_call_sync.reset)
        # initialize data for threadding

        # init timing data
        ### self.period = smo_default_period
        self.cycle_cnt = 0
        self.delay_time = 0
        self.start_time = 0
        self.stop_time = 0
        self.last_call = 0
        self.last_cnt = 0
        self.act_call = 0
        self.act_cnt = 0

        # create and start own sequencer
        self.sequencer = [SMOSequencer(self)]
        self.threaded = True
        # default start serialization
        if self.auto_serialize:
            self.sequencer[0].start_serialize()
            # default start periodic execution
            if self.auto_compute and self.period >= 0:
                self._start_compute()

        # enable remote calls
        self.thr_call_sync.set()
        return

# methods for serialization
    def is_serialized(self):
        self.thr_call_sync.wait(timeout=None)
        if self.threaded and self.sequencer[0] is not None:
            return self.sequencer[0].thr_serialize.isSet()
        else:
            if warning:
                mstr = '*** WARNING *** SMOSyncObject object no %s not threaded' % str(self.obj_no)
                self.debug_handler.out(mstr)
            return False

    def start_serialize(self):
        self.thr_call_sync.wait(timeout=None)
        if self.threaded and self.sequencer[0] is not None:
            return self.sequencer[0].start_serialize()
        else:
            if warning:
                mstr = '*** WARNING *** SMOSyncObject object no %s not threaded' % str(self.obj_no)
                self.debug_handler.out(mstr)
            return False

    def stop_serialize(self, thr_exit=False, thr_new_sequencer=None):
        self.thr_call_sync.wait(timeout=None)
        if not self.threaded or self.sequencer[0] is None:
            if warning:
                mstr = '*** WARNING *** SMOSyncObject object no %s not threaded' % str(self.obj_no)
                self.debug_handler.out(mstr)
            return False
        else:
            return self.sequencer[0].stop_serialize(thr_exit=thr_exit, thr_new_sequencer=thr_new_sequencer)

    def sync_call(self, method, *para_args, **para_kwd):

        # method call or send message (no return)
        call_type = 'Call'
        if 'smo_call_type' in para_kwd:
            call_type = para_kwd['smo_call_type']
            del para_kwd['smo_call_type']

        # no synchronisation
        if self.sequencer[0] is None:
            result = method(self, *para_args, **para_kwd)
            if call_type == 'Call':
                return result
            else:
                return None
        # syncronized call/send
# raises exeptions .... self ???

        SMOSyncObject.thr_call_sync.wait(timeout=None)
        ### thr_call( smo_sync_msg_type, smo_ack, smo_call, sync_obj, sync_method,  para_args,  para_kwd)
        return SMOSyncObject.sequencer[0].thr_call(call_type, None, method, para_args, para_kwd)

# periodic execution methods
    def is_computing(self):
        return self.threadded and self.run_compute

    def start_compute(self, **kwd):
        """Method for starting the computation of the node"""
        if self.threaded:
            res = self.sequencer[0].thr_call('Call', None, self._start_compute, (), kwd)
            return res
        else:
            if warning:
                mstr = '*** WARNING *** SMOSyncObject object no %s not threaded' % str(self.obj_no)
                self.debug_handler.out(mstr)
            return False

    def _start_compute(self, **kwd):
        """Methode which is called from THR class
        This method is is called during the serialization process"""
        """Method to determine the different times"""
        # Time may be passed by the simulation algorithm
        if self.trace:
            mstr = '*** TRACE *** SMOSyncObject _start_compute for objcet no %s' % str(self.obj_no)
            self.debug_handler.out(mstr)
        if self.run_compute:
            if self.warning:
                mstr = '*** WARNING *** SMOSyncObject _start_compute: object no  %s computation is still running' % str(
                    self.obj_no)
                self.debug_handler.out(mstr)
            return False
        else:
            start_time = smo_util.get_real_time()
            if self.stop_time <= 0:
                # first start after initialisation or reset
                self.start_time = start_time
                self.cycle_cnt = 0
                self.stop_time = 0
                self.delay_time = 0
                self.last_call = start_time
                self.last_cnt = 0
                self.act_call = start_time
                self.act_cnt = 0
            else:
                # restart after stop (break)
                delta = start_time - self.stop_time
                self.delay_time += delta
                self.last_call += delta
                self.act_call += delta
                self.stop_time = 0
            self.run_compute = True
            self.sequencer[0].thr_start = True
            return True

    def stop_compute(self, **kwd):
        if self.threaded:
            ### thr_call( smo_sync_msg_type, smo_ack, smo_call, sync_obj, sync_method,  para_args,  para_kwd)
            return self.sequencer[0].thr_call('Send', None, self._stop_compute, (), kwd)
        else:
            if warning:
                mstr = '*** WARNING *** SMOSyncObject object no %s not threaded' % str(self.obj_no)
                self.debug_handler.out(mstr)
            return False

    def _stop_compute(self, **kwd):
        if self.trace:
            mstr = '*** TRACE *** SMOSyncObject _stop_compute for obnject no %s' % str(self.obj_no)
            self.debug_handler.out(mstr)
        if not self.run_compute:
            if self.warning:
                mstr = '*** WARNING *** SMOSyncObject _stop_compute object no %s computation not started' % str(
                    self.obj_no)
                self.debug_handler.out(mstr)
            return False
        self.stop_time = smo_util.get_real_time()
        self.run_compute = False
        self.sequencer[0].thr_start = False
        return True

    def reset_compute(self, *arg, **kwd):
        if self.threaded:
            return self.sequencer[0].thr_call('Call', None, self._reset_compute, arg, kwd)
        else:
            if warning:
                mstr = '*** WARNING *** SMOSyncObject object no %s not threaded' % str(self.obj_no)
                self.debug_handler.out(mstr)
            return False

    def _reset_compute(self, *arg, **kwd):
        if self.trace:
            mstr = '*** TRACE *** SMOSyncObject _reset_compute for obnject no %s' % str(self.obj_no)
            self.debug_handler.out(mstr)
        self.thr[0].thr_start = False
        self.run_compute = False
        self.cycle_cnt = 0
        self.start_time = 0
        self.stop_time = 0
        self.last_call = 0
        self.last_cnt = 0
        self.act_call = 0
        self.act_cnt = 0
        return True

    def execute_compute(self, **kwd):
        # call compute handler
        """This method calculate the time data for the next period and call the computation handler
            time_tab ( <actual call number>, <actual call time>,  <last call number>, <last call time>)"""
        if not self.run_compute:
            if self.error:
                mstr = '*** ERROR *** SMOSyncObject execute compute: object no %s computation not started' % str(
                    self.obj_no)
                self.debug_handler.out(mstr)
        if self.trace:
            mstr = '*** TRACE *** SMOSyncObject execute_compute for object no %s' % str(self.obj_no)
            self.debug_handler.out(mstr)
        act_time = smo_util.get_real_time()
        if act_time < self.act_call + self.period:
            sleep_time = (self.act_call + self.period) - act_time
            if self.debug:
                mstr = '*** DEBUG *** SMOSyncObject execute_compute: object no %s sleep for %f seconds'\
                    % (str(self.obj_no), sleep_time)
                self.debug_handler.out(mstr)
            sleep(sleep_time)
            act_time = smo_util.get_real_time()
        self.last_cnt = self.act_cnt
        self.last_call = self.act_call
        self.act_cnt += 1
        self.act_call = act_time
        time_tab = {
            'start_time': self.start_time,
            'stop_time': self.stop_time,
            'act_cycle': self.act_cnt,
            'act_cycle_time': self.act_call,
            'last_cycle': self.last_cnt,
            'last_cycle_time': self.last_call}
        try:
            if self.debug:
                #mstr = '*** DEBUG *** SMOSyncObject execute_compute: object no %s\n   time table %s '\
                mstr = '*** DEBUG *** SMOSyncObject execute_compute for object no %s cycles %s, time %s'\
                    % (str(self.obj_no), str(time_tab['act_cycle'] - time_tab['last_cycle']), str(time_tab['act_cycle_time'] - time_tab['last_cycle_time']))
                self.debug_handler.out(mstr)
            self.compute_handler(time_tab=time_tab, **kwd)
        except:
            error_list = extract_tb(sys.exc_info()[2])
            error = error_list[len(error_list) - 1]
            if self.error:
                mstr = '*** ERROR *** SMOSyncObject execute compute: object no %s compute_handler raises an exception:\n  %s\n File: %s , Line number: %s ,\n   Method: %s , Statement: %s'\
                    % (str(self.obj_no), sys.exc_info()[1], error[0], error[1], error[2], error[3])
                self.debug_handler.out(mstr)
        return True

    def compute_handler(self, time_tab, **kwd):
        """
        This handler will be called for computation shouls be overwritten
        """
        if self.warning:
            mstr = '*** WARNING *** SMOSyncObject compute_handler for object no:  %s calling virtual method\n   time table time table: %s' % (
                str(self.obj_no), str(time_tab))
            self.debug_handler.out(mstr)
        return

# initialisation of the default sequencer


def smo_thr_start():
    if SMOBaseObject.info:
        smo_util.smo_debug_handler.out('*** INFO *** start global SMO sequencer')
    if SMOSyncObject.sequencer[0] is None:
        SMOSyncObject.sequencer[0] = SMOSequencer(SMOSyncObject)
        SMOSyncObject.thr_call_sync.set()
    ret = SMOSyncObject.sequencer[0].start_serialize()
    return ret


def smo_thr_stop():
    if SMOBaseObject.info:
        smo_util.smo_debug_handler.out('*** INFO *** stop global SMO sequencer')
    if SMOSyncObject.sequencer[0] is not None:
        SMOSyncObject.thr_call_sync.wait(timeout=None)
        SMOSyncObject.sequencer[0].stop_serialize(thr_exit=True)
        SMOSyncObject.sequencer[0] = None
    return

# set defaults
if smo_util.smo_threaded:
    if SMOBaseObject.info:
        print '*** INFO *** smo_sync_object: start sequencer'
    smo_thr_start()
else:
    print '*** INFO *** smo_sync_object: No global sequencer'
    SMOSyncObject.sequencer[0] = None

# stops gobal sequencer
register(smo_thr_stop)

# start main for testsequence
if __name__ == '__main__':
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ Start testsequence for SMOSyncObject')
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    smo_thr_start()

    class test_sync_class_1(SMOSyncObject):

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
            smo_util.smo_debug_handler.out('### test_sync_class time_tab: ', time_tab)
            return

        def print_data(self, para, n_para=''):
            smo_util.smo_debug_handler.out('### test_sync_1: %s, %s, %s, %s' % (
                str(para), str(n_para), str(self.data_1), str(self.parent)))
            return

        @smo_util.smo_sync_method_decorator
        def sync_call_prt(self, para, n_para=''):
            smo_util.smo_debug_handler.out('### sync_call_prt: %s, %s, %s, %s' %
                                           (str(para), str(n_para), str(self.data_1), str(self.parent)))
            return

        @smo_util.smo_sync_method_decorator
        def sync_send_prt(self, para, n_para='', wait=False):
            smo_util.smo_debug_handler.out('### sync_send_prt: %s, %s, %s, %s' %
                                           (str(para), str(n_para), str(self.data_1), str(self.parent)))
            return

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

    smo_util.smo_debug_handler.out('+++ create t_obj_1 (not threadded) and print it')
    t_obj_1 = test_sync_class_1()
    smo_util.smo_debug_handler.out('+++ ', str(t_obj_1))
    t_obj_1.print_data('para', n_para='otto')
###
##    smo_debug_handler.out( '\n+++++++++++++++++++++++++++++++++++')
##    smo_debug_handler.out( '+++ test smo_ipid')
##    smo_debug_handler.out( '+++ subscribe  t_obj_1 and print smo_ipid')
# t_obj_1.subscribe()
##    smo_debug_handler.out( '+++++++++++++++++++++++++++++++++++')
##    ipid_1 = t_obj_1.get_SMOid()
# print ipid_1
# print str(ipid_1)
##    smo_debug_handler.out (ipid_1)
##    smo_debug_handler.out (str(ipid_1))
##    smo_debug_handler.out( '+++ smo_id_str of t_obj_1: %s'%(t_obj_1.get_SMOid_str()))
##    ipid_2 = t_obj_1.get_SMOid()
##    smo_debug_handler.out( '+++ get object test: ', ipid_2.get_object() == t_obj_1)
##    smo_debug_handler.out( '+++ local smo_ip_call should proudce an error')
# ipid_2.ip_rpc_1(2,3,s='otto')
##    smo_debug_handler.out( '+++ local smo_ip_call')
# ipid_2.print_data(7,n_para='egon')
##    smo_debug_handler.out( '+++ local smo_ip_call should proudce an error')
# ipid_2.ip_rpc_1(2,3,s='otto')
##    smo_debug_handler.out( '+++ unsubscribe  t_obj_1')
# t_obj_1.unsubscribe()
##    smo_debug_handler.out( '+++ smo_obj_ref_dict: ', str(SMOIPID.smo_obj_ref_dict))
##    smo_debug_handler.out( '+++ smo_obj_id_dict: ', str(SMOIPID.smo_obj_id_dict))
###
    smo_util.smo_debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ create t_obj_2 (threadded)')
    t_obj_2 = test_sync_class_1(threaded=True, auto_serialize=False, auto_compute=False, period=1)
# auto_compute = True does not work properly
    smo_util.smo_debug_handler.out('+++ start serialization for obj %d ' % t_obj_2.obj_no)

    t_obj_2.start_serialize()
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ test thread calls')
    res = t_obj_2.sync_send_prt(12, n_para='otto')
    smo_util.smo_debug_handler.out('+++ result: ', str(res))
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    res = t_obj_2.sync_call_prt(15, n_para='willi')
    smo_util.smo_debug_handler.out('+++ result: ', str(res))
###

    smo_util.smo_debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ test recursive calls create objects 3, 4')

    t_obj_3 = test_sync_class_1(threaded=True, auto_serialize=True, auto_compute=False, period=0.5)
    t_obj_4 = test_sync_class_1(threaded=True, auto_serialize=True, auto_compute=False, period=0.3)
    ref_lst = (t_obj_1, t_obj_2, t_obj_3, t_obj_4)
    #ref_lst = ( t_obj_2,t_obj_3)

    smo_util.smo_debug_handler.out('+++ test recursive call level 0')
    t_obj_2.sync_recursive(call='otto', level=2, order=0, ref_lst=ref_lst)

    smo_util.smo_debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
    t_obj_2.sync_recursive(call='undef', level=0, order=0, ref_lst=ref_lst)
    smo_util.smo_debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
    t_obj_3.sync_recursive(call='undef', level=0, order=0, ref_lst=ref_lst)
    smo_util.smo_debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
    t_obj_4.sync_recursive(call='undef', level=0, order=0, ref_lst=ref_lst)

    smo_util.smo_debug_handler.out('+++ test recursive call level 2')
    t_obj_1.sync_recursive(call='undef', level=2, order=0, ref_lst=ref_lst)
    smo_util.smo_debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
    t_obj_2.sync_recursive(call='undef', level=2, order=0, ref_lst=ref_lst)
    smo_util.smo_debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
    t_obj_3.sync_recursive(call='undef', level=2, order=0, ref_lst=ref_lst)
    smo_util.smo_debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
    t_obj_4.sync_recursive(call='undef', level=2, order=0, ref_lst=ref_lst)


###
    smo_util.smo_debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ test periodic compute')
    smo_util.smo_debug_handler.out('+++ start computation for obj %d ' % t_obj_2.obj_no)
    t_obj_2.start_compute()
    s_time = 2

    smo_util.smo_debug_handler.out('+++ main sleep1 for %d seconds' % s_time)
    sleep(s_time)
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')

    res = t_obj_2.sync_send_prt(12, n_para='otto')
    smo_util.smo_debug_handler.out('+++ result: ', str(res))
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')

    smo_util.smo_debug_handler.out('+++ main sleep2 for %d seconds' % s_time)
    sleep(s_time)
    res = t_obj_2.sync_call_prt(15, n_para='willi')
    smo_util.smo_debug_handler.out('+++ result: ', str(res))
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')

    smo_util.smo_debug_handler.out('+++ main sleep3 for %d seconds' % s_time)
    sleep(s_time)
    smo_util.smo_debug_handler.out('+++ stop computation for obj %d ' % t_obj_2.obj_no)
    t_obj_2.stop_compute()

    sleep(s_time)

    smo_util.smo_debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ stop serialization for obj %d ' % t_obj_2.obj_no)
    t_obj_2.stop_serialize(thr_exit=True)
    t_obj_3.stop_serialize(thr_exit=True)
    t_obj_4.stop_serialize(thr_exit=True)
    # debug environment test
    smo_thr_stop()
    smo_util.smo_debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
    smo_util.smo_debug_handler.out('+++ End of test')
    smo_util.smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    exit
