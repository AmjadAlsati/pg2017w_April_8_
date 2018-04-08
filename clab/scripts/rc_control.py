#!/usr/bin/python2
################
#
# File:         rc_control.py
#
# Author:       Bernd Kleinjohann
# Ceated:      Oct. 2015
# Version:      1.0
#
# Contents:
"""
Remote control for robots
"""
from future import standard_library
standard_library.install_aliases()
from builtins import str

import os
import optparse

import tkinter as tk

from framework.smart_object import SMOBaseObject
from framework.remote_control import rc_util, init_ui
from framework.remote_control.rc_monitor import RCMonitor
from framework.remote_control.rc_init_node import RCInitTkGui


def main():
    if SMOBaseObject.application:
        SMOBaseObject.debug_handler.out('\n+++++++++++++++++++++++++++++++++++')
        SMOBaseObject.debug_handler.out('+++ Remote Control: START')
        SMOBaseObject.debug_handler.out('+++++++++++++++++++++++++++++++++++')

    # parse command line options
    p = optparse.OptionParser()
    p.add_option('--host', '-H', default='localhost')
    p.add_option('--port', '-P', type='int', default=8008)
    p.add_option('--path', '-p', default=None)
    p.add_option('--name', '-n', default=None)
    p.add_option('--control', '-c', default=rc_util.rc_default_serve)
    p.add_option('--server_port', '-s', default=rc_util.rc_default_server_port)
    p.add_option('--display', '-d', default=rc_util.rc_default_display)
    p.add_option('--threaded', '-t', default=False)
    p.add_option('--local', '-l', default=rc_util.rc_default_local)
    p.add_option('--gui', '-G', default=None)

    options, arguments = p.parse_args()

    if SMOBaseObject.info:
        SMOBaseObject.debug_handler.out(
            '*** info *** main: get following command line options:\n  path: %s   name %s\n   control: %s server_port: %s display: %s threaded: %s local: %s' % (
                str(options.path), str(options.name), str(options.control), str(options.server_port), str(options.threaded), str(options.display), str(options.local)))

    # get path and name of model for setting up;
    rc_path = options.path
    if options.path is None:
        rc_path = os.environ.get(rc_util.env_rc_path_name)
        if rc_path is None or not os.path.exists(rc_path):
            rc_path = os.getcwd()

    rc_name = options.name
    if rc_name is None:
        rc_name = os.environ.get(rc_util.env_rc_model_name)
        if rc_name is None:
            rc_name = rc_util.rc_default_model_name

    rc_fname = os.path.join(rc_path, '%s.py' % rc_name)
    rc_backup_fname = os.path.join(rc_path, '%s_back.py' % rc_name)

    if SMOBaseObject.info:
        SMOBaseObject.debug_handler.out('*** INFO *** main: try to open init_file %s' % rc_fname)

    RCInitTkGui.glob = globals()
    init_ui()

    rc_init_obj = RCInitTkGui(smo_init_mode='open', smo_filename=rc_fname, glob=globals())

    if rc_init_obj is None or rc_init_obj.valid == False:
        # create default rc init object and store it in robot_back_fname
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** WARNING *** main: try to create a new default initialization')
        rc_init_obj = RCInitTkGui()
        if rc_init_obj is None:
            if SMOBaseObject.fatal:
                SMOBaseObject.debug_handler.out('***\n*** fatal *** main: can not create any initialization\n***')
            tk.sys.exit()
        rc_init_obj.rc_name = rc_name
        rc_init_obj.rc_path = rc_path

    # create a copy of actual initialisation data
    if not rc_init_obj.smo_save(smo_filename=rc_backup_fname):
        if SMOBaseObject.warning:
            SMOBaseObject.debug_handler.out(
                '*** warning *** main: can not create a default init rc file in %s' % rc_backup_fname)

    if SMOBaseObject.application:
        SMOBaseObject.debug_handler.out(
            '+++ start with options:\n+++    path: %s; name %s;\n+++    control: %s; server_port: %s; display: %s; local: %s' % (
                str(rc_path), str(rc_name), str(options.control), str(options.server_port), str(options.display), str(options.local)))
        SMOBaseObject.debug_handler.out('+++++++++++++++++++++++++++++++++++')

# to do Kommuncation data
##
# if options.local: create local com class
# else create client and server
##
# if options.control or options.local:
# start control
##

    if options.display or options.local:
        # start of Gui
        rc_gui = RCMonitor(robot_host=options.host, robot_port=options.port, init_data=rc_init_obj, gui_filename=options.gui)
        # start monitor
        if options.threaded:
            if SMOBaseObject.application:
                SMOBaseObject.debug_handler.out('+++ Start RCMonitor in main')
            rc_gui.start_as_thread()
# wait for end??
        else:
            if SMOBaseObject.application:
                SMOBaseObject.debug_handler.out('+++ Start RCMonitor in main')
            rc_gui.start_in_main()

    if SMOBaseObject.application:
        SMOBaseObject.debug_handler.out('+++ Remote Control: END')
        SMOBaseObject.debug_handler.out('+++++++++++++++++++++++++++++++++++')
    return
# sys.exit()

if __name__ == '__main__':
    main()
