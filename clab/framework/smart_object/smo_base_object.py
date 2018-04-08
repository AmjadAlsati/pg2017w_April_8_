################
#
# Modul:        smart_object
# File:         smo_base_object.py
#
# Author:       Bernd Kleinjohann
# Created:      February 2014
# Version:      0.1
#
# Contents:     This File contains a base class for all SMOBaseObjects. An object may
#               become invalid,  if the user destroyed or delete a SMOBaseObject.
#               For identification a class global counter (id_count)  is used.
#               A unique id for a SMOBaseObject consists of a tupel (hash(self, self,id_count)
#
# To Do:        Handling of valid (external internal?) and object_no (skip ?)
#               Installing storing and restoring of methods
#               support of marshal and pickel
#

### global imports
import os
import sys
from traceback import extract_tb
from threading import Condition

from .smo_util import smo_msg_sel, smo_debug_handler


class SMOBaseObject:
    """This File contains a base class for all SMO Objects """
    # Counts the number of SMOBaseObjects which were created since start of the system
    id_lock = Condition(lock=None)
    """ Thread syncronisation for modifying id_count"""
    id_count = -1
    """ Count for unique identification of a SMO object. If the user creates a
         new SMOBaseObject, this counter will be incremented,"""

    # process unique id number for call return etc. messages
    msg_id_lock = Condition(lock=None)
    """ Thread syncronisation for modifying IP_call_id_count"""
    msg_id_count = 0
    """unique id number for RPC messages"""

    # debug class for the SMO system defined in smo_util
    debug_handler = smo_debug_handler
    """The SMO debug class sends synchronized error messages to an out channel, default is synchronised print"""
    # SMO global debug options, overwrite by class options possible
    # Default defined in smo_util
    application = smo_msg_sel['application']
    """Class filter option for debug handling"""
    debug = smo_msg_sel['debug']
    """Class Filter option for debug handling"""
    test = smo_msg_sel['test']
    """Class filter option for test handling"""
    trace = smo_msg_sel['trace']
    """Class filter option for trace handling"""
    verbous = smo_msg_sel['verbous']
    """Class filter option for verbose fault messages"""
    info = smo_msg_sel['info']
    """Class filter option for info handling"""
    warning = smo_msg_sel['warning']
    """Class filter option for warning handling"""
    error = smo_msg_sel['error']
    """Filter option for error handling"""
    fatal = smo_msg_sel['fatal']
    """Class filter option for fatal error handling"""

    def __init__(self, smo_init_mode='new', smo_restore=None,
                 smo_filename=None, smo_file=None, glob=None, **kwd):
        """Initalisation of SMOBaseObject, glob should contain all class definitions for the restored object (typcally globals())"""
        self.valid = False
        """Flag if object is valid or destroyed by the SMO system """

        self.glob = glob
        if self.glob is None:
            self.glob = globals()

        # create a new object
        if smo_init_mode == 'new':
            self.id_lock.acquire(blocking=1)
            SMOBaseObject.id_count += 1
            self.obj_no = SMOBaseObject.id_count
            self.id_lock.release()
            # if self.trace:
            if self.trace:
                mstr = '*** TRACE *** SMOBaseObject init: Type: %s  obj_no %s smo_init_mode %s'\
                    % (self.__class__.__name__, str(self.obj_no), str(smo_init_mode))
                self.debug_handler.out(mstr)
            self.valid = True
            return

        # restore an object based on a local dict in smo_restore
        elif smo_init_mode == 'restore':
            if not isinstance(smo_restore, dict):
                if self.fatal:
                    mstr = '*** FATAL *** SMOBaseObject init: Type %s get no restore dict'\
                        % (self.__class__.__name__)
                    self.debug_handler.out(mstr)
                    self.valid = False
                return
            self.__dict__.update(smo_restore)
            self.id_lock.acquire(blocking=1)
            SMOBaseObject.id_count += 1
            self.obj_no = SMOBaseObject.id_count
            self.id_lock.release()
            if self.trace:
                mstr = '*** TRACE *** SMOBaseObject init: Type%s  obj_no %s smo_init_mode %s'\
                    % (self.__class__.__name__, str(self.obj_no), str(smo_init_mode))
                self.debug_handler.out(mstr)
            self.valid = True
            return

        # restore object from a file
        elif smo_init_mode == 'open' or smo_init_mode == 'rewrite':
            if smo_init_mode == 'open' and smo_filename is None or\
               smo_init_mode == 'rewrite' and smo_file is None:
                if self.error:
                    mstr =  '*** ERROR *** SMOBaseObject init: Type %s smo_init_mode %s get no smo_file(name)'\
                        % (self.__class__.__name__, str(smo_init_mode))
                    self.debug_handler.out(mstr)
                return
            # read file and evaluate python code
            try:
                if smo_init_mode == 'open':
                    object_file = open(smo_filename, mode='r+')
                else:
                    object_file = smo_file
                restore_str = object_file.read()
                if smo_init_mode == 'open':
                    object_file.close()
                if self.glob is not None:
                    restore_dict = eval(restore_str, glob)
                else:
                    restore_dict = eval(restore_str)
                self.__dict__.update(restore_dict)
                self.id_lock.acquire(blocking=1)
                SMOBaseObject.id_count += 1
                self.obj_no = SMOBaseObject.id_count
                self.id_lock.release()
                if self.trace:
                    mstr = '*** TRACE *** SMOBaseObject init: Type%s  obj_no %s smo_init_mode %s'\
                        % (self.__class__.__name__, str(self.obj_no), str(smo_init_mode))
                    self.debug_handler.out(mstr)
                self.restore_ref(**kwd)
                self.valid = True
                return
            except:
                if self.info:
                    error_list = extract_tb(sys.exc_info()[2])
                    error = error_list[len(error_list) - 1]
                    mstr = '*** ERROR *** SMOBaseObject init: obj_no : Could not read file'
                    mstr += '\n    Exception: %s\n    File: %s , Line number: %s ,\n    Method: %s , Statement: %s'\
                        % (sys.exc_info()[1], error[0], error[1], error[2], error[3])
                    self.debug_handler.out(mstr)
                self.valid = False
                return None
        else:
            if self.error:
                mstr = '*** ERROR *** SMOBaseObject init: Type %s obj_no %s;  illegal smo_init_mode %s'\
                    % (self.__class__.__name__, str(self.obj_no), str(smo_init_mode))
                self.debug_handler.out(mstr)
            self.valid = False
            return

    def get_restore_dict(self):
        """
        This method should return a dictionary which contain all persistent data
        for all members of restore eval(repr(restore)) has to work
        """
        retdict = {}
        if self.trace:
            mstr = '*** TRACE *** SMOBaseObject get_restore_dict: return %s' % str(retdict)
            self.debug_handler.out(mstr)
        return retdict

    def restore_ref(self, *a, **kwd):
        if self.trace:
            self.debug_handler.out('*** TRACE *** SMOBaseObject restore_ref:')
        return

    def __repr__(self):
        """
        This method create a string in python syntax, which can reconstruct an
        instance of a smo_Object even if a spezial class is derived
        internal the method GetRestoreDict dict is used
        """
        if self.trace:
            mstr = '*** TRACE *** SMOBaseObject __repr__'
            self.debug_handler.out(mstr)

        retstr = '\n### Begin of SMOBaseObject of Type: ' + self.__class__.__name__ + '\n'
        retstr += self.__class__.__name__ + '( smo_init_mode = \'restore\',smo_restore =' + '\\' + '\n'
        retstr += repr(self.get_restore_dict())
        retstr += ')\\' + '\n'
        retstr += '### End of SMOBaseObject of Type: ' + self.__class__.__name__ + '\n'
        return retstr

    def __str__(self):
        """Produces a string containing the type of the SMOBaseObject"""
        # This string will be extended by subclasses
        if self.valid:
            return 'SMOBaseObject %s of type %s;' % (str(self.obj_no), self.__class__.__name__)
        else:
            return 'Invalid SMOBaseObject %s of Type %s;' % (str(self.obj_no), self.__class__.__name__)

    def smo_save(self, smo_filename=None, smo_file=None, smo_save_mode='save'):
        if smo_save_mode != 'save' and smo_save_mode != 'rewrite':
            if self.error:
                mstr = '*** ERROR *** SMOBaseObject smo_save: Type %s obj_no %s  illegal smo_save_mode %s'\
                    % (self.__class__.__name__, str(self.obj_no), smo_save_mode)
                self.debug_handler.out(mstr)
            return False
        if smo_save_mode == 'save' and smo_filename is None or\
           smo_save_mode == 'rewrite' and smo_file is None:
            if self.error:
                mstr = '*** ERROR *** SMOBaseObject smo_save: Type %s obj_no %s; get no smo_file(name)'\
                    % (self.__class__.__name__, str(self.obj_no), smo_save_mode)
                self.debug_handler.out(mstr)
            return False
        # store data in file or in fllename
        try:
            # get the python representation
            restore_str = repr(self.get_restore_dict())
            # simple formating
            restore_str = restore_str.replace(', ', ',\\\n')
            # try to get a name of the object for documentation purpouse
            try:
                name = self.name
            except:
                name = '-unknown-'
            # create a simple header
            header_str = '###########################\n'
            header_str += '###\n'
            header_str += '### SMOBaseObject no. %d of type %s and name %s \n###\n'\
                % (self.obj_no, self.__class__.__name__, name)
            header_str += '###\n'
            # create a simple postscriptum
            ps_str = '\n###\n'
            ps_str += '### SMOBaseObject end\n###\n'
            ps_str += '###########################\n'
            # prepaire the file data
            restore_str = header_str + restore_str + ps_str
            # write filedata
            if smo_save_mode == 'save':
                object_file = open(smo_filename, mode='w+')
                object_file.write(restore_str)
                object_file.close()
            else:
                object_file.seek(0)
                object_file.write(restore_str)
            return True
        except:
            if self.fatal:
                #            if True:
                error_list = extract_tb(sys.exc_info()[2])
                error = error_list[len(error_list) - 1]
                mstr = '*** FATAL *** SMOBaseObject smo_save: obj_no %s: Could not write file' % (str(self.obj_no))
                mstr += '\n    Exception: %s\n    File: %s , Line number: %s ,\n    Method: %s , Statement: %s'\
                    % (sys.exc_info()[1], error[0], error[1], error[2], error[3])
                self.debug_handler.out(mstr)
            return False

    # compatibility ??? get_id(self):  return (hash(self), self.id_count)

    def get_obj_no(self):
        """This method checks if the SMOBaseObject is valid and returns an id (unique obj_no)"""
        if self.valid:
            return self.obj_no
        else:
            return -1

    def is_valid(self):
        return self.valid

    def set_invalid(self):
        self.valid = False

        return

    def set_instance_debug_options(self, **kwd):
        """This Method handels a debug filter for an instance of a class derived from SMOBaseObject
        The method is inherited by all subclasses and can therefore be used
        by every instance individually"""
        for x in kwd:
            if 'application' == x:
                self.application = kwd['application']
                if x is None and 'application' in self.__dict__:
                    del self.application
            elif 'debug' == x:
                self.debug = kwd['debug']
                if x is None and 'debug' in self.__dict__:
                    del self.debug
            elif 'trace' == x:
                self.trace = kwd['trace']
                if x is None and 'trace' in self.__dict__:
                    del self.trace
            elif 'verbous' == x:
                self.verbous = kwd['verbous']
                if x is None and 'verbous' in self.__dict__:
                    del self.verbous
            elif 'info' == x:
                self.info = kwd['info']
                if x is None and 'info' in self.__dict__:
                    del self.info
            elif 'warning' == x:
                self.warning = kwd['warning']
                if x is None and 'warning' in self.__dict__:
                    del self.warning
            elif 'error' == x:
                self.error = kwd['error']
                if x is None and 'error' in self.__dict__:
                    del self.error
            elif 'fatal' == x:
                self.fatal = kwd['fatal']
                if x is None and 'fatal' in self.__dict__:
                    del self.fatal
            else:
                if self.warning:
                    mstr = '*** ERROR *** SMOBaseObject set_instance_debug_options: Type %s obj_no %s; smo_init_mode %s, illegal smo_init_mode %s, illegal debug mode %s'\
                        % (self.__class__.__name__, str(self.obj_no), str(smo_init_mode), str(x))
                    self.debug_handler.out(mstr)
        return

    def set_class_debug_options(self, **kwd):
        """This Method handels a debug filter for a class derived from SMOBaseObject
        The method is inherited by all subclasses and can therefore be used
        by every subclass individually"""
        for x in kwd:
            if 'application' == x:
                self.__class__.application = kwd['application']
                if x is None and self.__class__ != SMOBaseObject and\
                   'application' in self.__class__.__dict__:
                    del self.__class__.application
            elif 'debug' == x:
                self.__class__.debug = kwd['debug']
                if x is None and self.__class__ != SMOBaseObject and\
                   'debug' in self.__class__.__dict__:
                    del self.__class__.debug
            elif 'trace' == x:
                self.__class__.trace = kwd['trace']
                if x is None and self.__class__ != SMOBaseObject and\
                   'trace' in self.__class__.__dict__:
                    del self.__class__.trace
            elif 'verbous' == x:
                self.__class__.verbous = kwd['verbous']
                if x is None and self.__class__ != SMOBaseObject and\
                   'verbous' in self.__class__.__dict__:
                    del self.__class__.verbous
            elif 'info' == x:
                self.__class__.info = kwd['info']
                if x is None and self.__class__ != SMOBaseObject and\
                   'info' in self.__class__.__dict__:
                    del self.__class__.info
            elif 'warning' == x:
                self.__class__.warning = kwd['warning']
                if x is None and self.__class__ != SMOBaseObject and\
                   'warning' in self.__class__.__dict__:
                    del self.__class__.warning
            elif 'error' == x:
                self.__class__.error = kwd['error']
                if x is None and self.__class__ != SMOBaseObject and\
                   'error' in self.__class__.__dict__:
                    del self.__class__.error
            elif 'fatal' == x:
                self.__class__.fatal = kwd['fatal']
                if x is None and self.__class__ != SMOBaseObject and\
                   'fatal' in self.__class__.__dict__:
                    del self.__class__.fatal
            else:
                if self.warning:
                    mstr = '*** ERROR *** SMOBaseObject set_class_debug_optinons: Type %s obj_no %s; smo_init_mode %s, illegal smo_init_mode %s, illegal debug mode %s'\
                        % (self.__class__.__name__, str(self.obj_no), str(smo_init_mode), str(x))
                    self.debug_handler.out(mstr)
        return

    def set_debug_options(self, **kwd):
        """This Method handels a debug filter for SMOBaseObject instances
        The method is inherited by all subclasses and can therefore be used
        by every subclass individually"""
        for x in kwd:
            if 'application' == x:
                SMOBaseObject.application = kwd['application']
            elif 'debug' == x:
                SMOBaseObject.debug = kwd['debug']
            elif 'trace' == x:
                SMOBaseObject.trace = kwd['trace']
            elif 'verbous' == x:
                SMOBaseObject.verbous = kwd['verbous']
            elif 'info' == x:
                SMOBaseObject.info = kwd['info']
            elif 'warning' == x:
                SMOBaseObject.warning = kwd['warning']
            elif 'error' == x:
                SMOBaseObject.error = kwd['error']
            elif 'fatal' == x:
                SMOBaseObject.fatal = kwd['fatal']
            else:
                if self.warning:
                    mstr = '*** ERROR *** SMOBaseObject set_debug_options: Type %s obj_no %s; illegal debug mode %s'\
                        % (self.__class__.__name__, str(self.obj_no), str(x))
                    self.debug_handler.out(mstr)
        return


def smo_set_debug_options(**kwd):
    """This Method handels a debug filter for a class instance of all SMOBaseObject instances
    The method is inherited by all subclasses and can therefore be used
    by every subclass individually"""
    for x in kwd:
        if 'application' == x:
            SMOBaseObject.application = kwd['application']
        elif 'debug' == x:
            SMOBaseObject.debug = kwd['debug']
        elif 'trace' == x:
            SMOBaseObject.trace = kwd['trace']
        elif 'verbous' == x:
            SMOBaseObject.verbous = kwd['verbous']
        elif 'info' == x:
            SMOBaseObject.info = kwd['info']
        elif 'warning' == x:
            SMOBaseObject.warning = kwd['warning']
        elif 'error' == x:
            SMOBaseObject.error = kwd['error']
        elif 'fatal' == x:
            SMOBaseObject.fatal = kwd['fatal']
        else:
            if SMOBaseObject.warning:
                mstr = '*** WARNING *** smo_set_debug_options: Illegal debug mode %s' % str(x)
                SMOBaseObject.debug_handler.out(mstr)
    return


def smo_open(smo_filename=None, glob=None, **kwd):
    if glob is None:
        glob = globals()
    try:
        object_file = open(smo_filename, mode='r+')
    except:
        error_list = extract_tb(sys.exc_info()[2])
        error = error_list[len(error_list) - 1]
        if SMOBaseObject.error:
            SMOBaseObject.debug_handler.out(
                '*** ERROR *** SMOOpen: Exception during open File %s:\n   %s'
                % (str(smo_filename), sys.exc_info()[1]))
        return None
    try:
        restore_str = object_file.read()
        if glob is not None:
            obj = eval(restore_str, glob)
        else:
            obj = eval(restore_str)
            obj.glob = glob
        obj.id_lock.acquire(blocking=1)
        SMOBaseObject.id_count += 1
        obj.obj_no = SMOBaseObject.id_count
        obj.id_lock.release()

        # restore ref
        obj.restore_ref(**kwd)
        if SMOBaseObject.trace:
            SMOBaseObject.debug_handler.out(
                '*** TRACE *** SMOOpen file: %s, Type: %s' % (smo_filename, obj.__class__.__name__))
        return obj
    except:
        error_list = extract_tb(sys.exc_info()[2])
        error = error_list[len(error_list) - 1]
        if SMOBaseObject.error:
            SMOBaseObject.debug_handler.out(
                '*** ERROR *** SMOOpen: Exception during construction, File %s:\n   %s\n   File: %s , Line number: %s ,\n   Method: %s , Statement: %s'
                % (smo_filename, sys.exc_info()[1], error[0], error[1], error[2], error[3]))
        return None


def smo_save(obj=None, smo_filename=None):
    try:
        object_file = open(smo_filename, mode='w+')
    except:
        error_list = extract_tb(sys.exc_info()[2])
        error = error_list[len(error_list) - 1]
        if SMOBaseObject.error:
            SMOBaseObject.debug_handler.out(
                '*** ERROR *** SMOSave: Exception during open File %s:\n   %s'
                % (str(smo_filename), sys.exc_info()[1]))
        return False
    try:
        # get the python representation
        restore_str = repr(obj.get_restore_dict())
        # simple formating
        restore_str = restore_str.replace(', ', ',\\\n')
        # create a simple header

        header_str = '###########################\n'
        header_str += '###\n'
        header_str += '### SMOBaseObject of Type %s \n'\
                      % obj.__class__.__name__
        header_str += '###\n'
        header_str += obj.__class__.__name__ + '( smo_init_mode = \'restore\',smo_restore =' + '\\' + '\n'
        # create a simple postscriptum
        ps_str = ')\\' + '\n'
        ps_str += '### End of SMOBaseObject of Type' + obj.__class__.__name__ + '\n'
        ps_str += '###########################\n'
        # prepaire the file data
        restore_str = header_str + restore_str + ps_str
        # write filedata
        object_file.write(restore_str)
        object_file.close()
        return True
    except:
        error_list = extract_tb(sys.exc_info()[2])
        error = error_list[len(error_list) - 1]
        if SMOBaseObject.error:
            SMOBaseObject.debug_handler.out(
                '*** ERROR *** SMOSave: Exception during save:\n   %s\n   File: %s , Line number: %s , Method: %s , Statement: %s' %
                (sys.exc_info()[1], error[0], error[1], error[2], error[3]))
        return False

# start main for testsequence
if __name__ == '__main__':

    class test_class_1(SMOBaseObject):

        def __init__(self, parent=None, smo_init_mode='new', **kwd):
            self.data_1 = 'test_init_error'
            SMOBaseObject.__init__(self, smo_init_mode=smo_init_mode, **kwd)
            if smo_init_mode == 'new':
                self.data_1 = 'test_data'
                self.parent = parent
            return

        def get_restore_dict(self):
            ret_dict = SMOBaseObject.get_restore_dict(self)
            local_dict = {'data_1': self.data_1}
            ret_dict.update(local_dict)
            return ret_dict

        def restore_ref(self, test_para='', test_parent=None, **kwd):
            SMOBaseObject.restore_ref(self)
            if test_parent is None:
                print '+++ restore ref ', test_para, str(test_parent)
            else:
                print '+++ restore ref ', test_para
            return

        def print_data(self, para, n_para=''):
            print 'test_class_1:', str(para), str(n_para), self.data1, self.parent
            return

        def debug_test(self):
            if self.application:
                mstr = '+++ Application message occure self: %s, instance: %s, class: %s' % (
                    self.application, test_class_1.application, SMOBaseObject.application)
                self.debug_handler.out(mstr)

        def set_data(self, para):
            self.data_1 = para
            return

        def get_data(self): return self.data1

    class test_class_2(test_class_1):

        def __init__(self, smo_init_mode='new', **kwd):
            self.data_2 = 'another_test_data'
            self.data_3 = None
            test_class_1.__init__(self, smo_init_mode=smo_init_mode, **kwd)
            if smo_init_mode == 'new':
                self.data_2 = 'another_test_data'
                self.data_3 = test_class_1(parent=self)
            return

        def get_restore_dict(self):
            ret_dict = test_class_1.get_restore_dict(self)
            local_dict = {'data_2': self.data_2,
                          'data_3': self.data_3}
            ret_dict.update(local_dict)
            return ret_dict

        def restore_ref(self, test_para='', test_parent=None, **kwd):
            test_class_1.restore_ref(self, test_para=test_para, test_parent=test_parent)
            self.data_3.restore_ref(test_para=test_para, test_parent=self)
            return

        def print_data(self, para, n_para=''):
            print 'test_class_2:', str(para), str(n_para), self.data_2, self.data_3
            return

        def set_data(self, para):
            self.data_1 = para
            return

        def get_data(self): return self.data_2

    smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    smo_debug_handler.out('+++ Start Testsequence for SMOBaseObject')
    smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')

    smo_set_debug_options(trace=False)

    # SMOBaseObject test
    test_obj = SMOBaseObject()
    smo_debug_handler.out('+++ create test_obj (SMOBaseClass) Obj. No.: ',
                          test_obj.get_obj_no(), ' Valid:', test_obj.is_valid())
    smo_debug_handler.out('+++ print test_obj')
    print test_obj
    smo_debug_handler.out('+++ make test_obj invalid')
    test_obj.set_invalid()
    smo_debug_handler.out('+++ print test_obj')
    print test_obj
    smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')

    # debug test
    # smo_set_debug_options(application=False)
    test_obj_1 = test_class_1()
    smo_debug_handler.out('+++ create test_obj_1 (test_class_1, debug) Obj. No.: ',
                          test_obj_1.get_obj_no(), ' Valid:', test_obj_1.is_valid())

    smo_debug_handler.out(
        'test debug option, smo_set_debug_options sets application to False, no Application message should occure')
    smo_debug_handler.out('+++ print application message')
    test_obj_1.debug_test()
    smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    smo_set_debug_options(application=False)
    smo_debug_handler.out(
        'test debug option, smo_set_debug_options sets application to False, no Application message should occure')
    test_obj_1.debug_test()
    smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    test_obj_1.set_class_debug_options(application=True)
    smo_debug_handler.out(
        'test debug option, smo_set_class_debug_options sets application to True, Application message should occure')
    test_obj_1.debug_test()
    smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    test_obj_1.set_instance_debug_options(application=False)
    smo_debug_handler.out(
        'test debug option, smo_set_instance_debug_options sets application to False, no Application message should occure')
    test_obj_1.debug_test()
    smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
    test_obj_1.set_instance_debug_options(application=None)
    smo_debug_handler.out(
        'test debug option, smo_set_instance_debug_options sets application to None, Application message should occure')
    test_obj_1.debug_test()
    smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')

    # persistance test
    smo_debug_handler.out('+++ create test_obj_2 , test smo_open, smo_save functions ')
    test_data_path = os.getcwd()
    test_fname = os.path.join(test_data_path, 'smo_test1_data.py')
    smo_debug_handler.out('+++ use file %s for testing' % test_fname)
    test_obj_2 = test_class_2()
    smo_debug_handler.out('+++ create test_obj_2 (test_class_2) ', test_obj_2.get_obj_no(), ' ', test_obj_2.is_valid())
    smo_save(obj=test_obj_2, smo_filename=test_fname)
    test_obj_2_restore = smo_open(smo_filename=test_fname, glob=globals(),
                                  test_para='restore_ref_para', test_parent=None)
    smo_debug_handler.out('+++ print test_obj_2_restore ', test_obj_2_restore.get_obj_no(),
                          ' ', test_obj_2_restore.is_valid())
    print test_obj_2_restore
    smo_debug_handler.out('+++ create test_obj_2 , test methods smo_open, smo_save functions ')
    test_fname_2 = os.path.join(test_data_path, 'smo_test2_data.py')
    test_obj_2.smo_save(smo_filename=test_fname_2, smo_save_mode='save')
    test_obj_3 = test_class_2(smo_init_mode='open', smo_filename=test_fname_2,
                              glob=globals(), test_para='restore_ref_para', test_parent=None)
    smo_debug_handler.out('+++ print test_obj_3_ ', test_obj_3.get_obj_no(), ' ', test_obj_3.is_valid())
    print test_obj_3

    smo_debug_handler.out('+++ make test_obj_3 invalid')
    test_obj_3 = SMOBaseObject()
    print test_obj_3
    smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')

    smo_debug_handler.out('+++ End of Testsequence')
    smo_debug_handler.out('+++++++++++++++++++++++++++++++++++')
