################
#
# Modul:        PaderKicker Architekture
# File:              smo_util.py
#
# Author:       Bernd Kleinjohann
# Created:    November 2008
# Version:      1.0
#
# Contents:
"""
This file contains global setups, configurations and a simple Error/debug handler
"""

import timeit
import threading

# defines the operating system dependent timebase
get_real_time = timeit.default_timer
"""Sets the timer depending on the system"""

# Alternative for timeit function selection
"""if sys.platform == "win32":
    ### On Windows, the best timer is time.clock()
    default_timer = time.clock
else:
    ### On most other platforms the best timer is time.time()
    default_timer = time.time"""

# SMO global debug options, overwrite by class or instance options possible
#smo_msg_sel = {'application':True,'debug':True, 'test':True, 'trace':True, 'verbous':True, 'info':True, 'warning':True, 'error':True, 'fatal':True}
smo_msg_sel = {'application': False, 'debug': False, 'test': False, 'trace': False,
               'verbous': False, 'info': True, 'warning': True, 'error': True, 'fatal': True}
"""Global debuging options"""

smo_application = smo_msg_sel['application']
smo_debug = smo_msg_sel['debug']
smo_test = smo_msg_sel['test']
smo_trace = smo_msg_sel['trace']
smo_verbous = smo_msg_sel['verbous']
smo_info = smo_msg_sel['info']
smo_warning = smo_msg_sel['warning']
smo_error = smo_msg_sel['error']
smo_fatal = smo_msg_sel['fatal']

# start configuration of the esm top level (anchor)
#smo_threaded = True
smo_threaded = False
smo_serialize = True
smo_compute = False  # ???
# defaults for creating ESMThrObjects
smo_default_threaded = False
smo_default_serialize = True
smo_default_autocompute = False
smo_default_period = 1

# environment var for path and all init data
#smo_env_path = 'smo_DATA_PATH'
#smo_default_fname = 'smo_root_data.py'
smo_server_id_str = 'smo_server'
smo_client_id_str = 'smo_client'
smo_default_id_str = 'smo_object'

# legal scope of SMO ports
smo_IP_min_port = 5000
smo_IP_max_port = 10000
smo_IP_default_server_port = 5562
smo_IP_default_reconnect_period = 2
#smo_IP_default_period = 0.5
smo_IP_default_server_host = 'localhost'
smo_IP_msg_length = 8192

# String for seperation of ip messages
# format descriptor in python source ('format', size = -1)
# format : py_source, ...
# size -1: variable length
smo_IP_separator = '\n#*+SMO+*#\n'
smo_IP_data_separator = '\n#*+smo_DATA+*#\n'
"""String for seperation of ip messages and message datadata"""


# definitions of esm exceptions
class SMOExeption(Exception):
    """Base Exeption for smo module"""


class SMOFatal(SMOExeption):
    """System inconsistent"""


class SMOError(SMOExeption):
    """Thomething goes wrong"""


class SMOConnection(SMOExeption):
    """Connection problems"""

# decorators for thread and rpc synchronisation of SMOSyncObjects


def smo_sync_method_decorator(method):
    def _aux_sync_call(self, *para_args, **para_kwd):
        # method call or send message (no return)
        call_type = 'Call'
        if 'smo_call_type' in para_kwd:
            call_type = para_kwd['smo_call_type']
            del para_kwd['smo_call_type']
        # no synchronisation
        if self.sequencer[0] is None:
            # raises exeptions .... self ???
            result = method(self, *para_args, **para_kwd)
            if call_type == 'Call':
                return result
            else:
                return None
        # syncronized call/send
        self.thr_call_sync.wait(timeout=None)
        return self.sequencer[0].thr_call(call_type, self, method, para_args, para_kwd)
    return _aux_sync_call


def smo_sync_function_decorator(func):
    def _aux_sync_call(*para_args, **para_kwd):
        # method call or send message (no return)
        call_type = 'Call'
        if 'smo_call_type' in para_kwd:
            call_type = para_kwd['smo_call_type']
            del para_kwd['smo_call_type']
        # no synchronisation
        if SMOSyncObject.sequencer[0] is None:
            # raises exeptions .... self ???
            result = func(*para_args, **para_kwd)
            if call_type == 'Call':
                return result
            else:
                return None
        # syncronized call/send
        SMOSyncObject.thr_call_sync.wait(timeout=None)
        ### thr_call( smo_sync_msg_type, smo_ack, smo_call, sync_obj, sync_method,  para_args,  para_kwd)
        return SMOSyncObject.sequencer[0].thr_call(call_type, None, func, para_args, para_kwd)
    return _aux_sync_call


class SMODebug:
    """This class is used for preliminary debug and error handling"""

    debug_lock = threading.Condition(lock=None)
    """Synchronisation for debug messages of different threads"""

    def __init__(self, dest=None, **kwd):
        self.dest = dest
        """Up to now dest should be None, further extention is possible
        Default uses synchronized  print, ** kwd not used up to now"""
        return

    def out(self, *para, **kwd):
        """Prints the messages thread-safe"""
        self.debug_lock.acquire(blocking=1)
        # thread synchronisation for print
        if self.dest is None:
            msg = ''
            for x in para:
                msg += x.__str__()
        print msg
        self.debug_lock.release()
        return

# make debug handler available for main
smo_debug_handler = SMODebug()
